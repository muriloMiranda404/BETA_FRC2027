package frc.frc_java9485.utils.logger.wpilogxz;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;
import org.littletonrobotics.junction.LogTable.LoggableType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;


public class WPILOGXZWriter implements LogDataReceiver {


    private static final double TIMESTAMP_UPDATE_DELAY_SEC = 5.0;


    private static final long FLUSH_PERIOD_US = 250000L;

    private static final String DEFAULT_PATH_RIO = "/U/logs";
    private static final String DEFAULT_PATH_SIM = "logs";
    private static final String ADVANTAGE_SCOPE_FILE_NAME = "ascope-log-path.txt";
    private static final DateTimeFormatter TIME_FORMATTER = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss");

    private final String randomIdentifier;
    private final AdvantageScopeOpenBehavior openBehavior;

    private String folder;
    private String filename;
    private boolean autoRename;

    private Double dsAttachedTime;
    private LocalDateTime logDate;
    private String logMatchText;

    private WPILOGXZEncoder encoder;
    private FileOutputStream fileOutputStream;
    private boolean isOpen = false;

    private LogTable lastTable;
    private int timestampID;
    private long lastFlushTimestamp = 0;

    private Map<String, Integer> entryIDs;
    private Map<String, LoggableType> entryTypes;
    private Map<String, String> entryUnits;


    public WPILOGXZWriter(String path, AdvantageScopeOpenBehavior openBehavior) {
        this.openBehavior = openBehavior;

        Random random = new Random();
        StringBuilder identifier = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            identifier.append(String.format("%04x", random.nextInt(0x10000)));
        }
        this.randomIdentifier = identifier.toString();

        if (path.endsWith(".wpilog") || path.endsWith(".wpilogxz")) {
            File pathFile = new File(path);
            this.folder = pathFile.getParent();
            this.filename = pathFile.getName();
            this.autoRename = false;
        } else {
            this.folder = path;
            this.filename = "akit_" + randomIdentifier + ".wpilogxz";
            this.autoRename = true;
        }
    }

    public WPILOGXZWriter(String path) {
        this(path, AdvantageScopeOpenBehavior.AUTO);
    }

    public WPILOGXZWriter(AdvantageScopeOpenBehavior openBehavior) {
        this(RobotBase.isSimulation() ? DEFAULT_PATH_SIM : DEFAULT_PATH_RIO, openBehavior);
    }

    public WPILOGXZWriter() {
        this(RobotBase.isSimulation() ? DEFAULT_PATH_SIM : DEFAULT_PATH_RIO, AdvantageScopeOpenBehavior.AUTO);
    }

    @Override
    public void start() {
        File logFolder = new File(folder);
        if (!logFolder.exists()) {
            logFolder.mkdirs();
        }

        File logFile = new File(folder, filename);
        if (logFile.exists()) {
            logFile.delete();
        }

        String logPath = Path.of(folder, filename).toString();
        System.out.println("[AdvantageKit] Logging to \"" + logPath + "\" (LZMA2 compressed)");

        try {
            fileOutputStream = new FileOutputStream(logPath);
            encoder = new WPILOGXZEncoder(new BufferedOutputStream(fileOutputStream));
            encoder.writeHeader(WPILOGConstants.EXTRA_HEADER);

            timestampID = encoder.startEntry(
                    timestampKey,
                    LoggableType.Integer.getWPILOGType(),
                    WPILOGConstants.ENTRY_METADATA,
                    0);
        } catch (IOException e) {
            DriverStation.reportError("[AdvantageKit] Failed to open output log file.", true);
            return;
        }

        isOpen = true;
        lastTable = new LogTable(0);

        entryIDs = new HashMap<>();
        entryTypes = new HashMap<>();
        entryUnits = new HashMap<>();
        logDate = null;
        logMatchText = null;
    }

    @Override
    public void end() {
        if (!isOpen) {
            return;
        }

        try {
            flush();
        } catch (IOException e) {
            DriverStation.reportError("[AdvantageKit] Failed to flush log file.", false);
        }

        try {
            encoder.close();
            isOpen = false;
        } catch (IOException e) {
            DriverStation.reportError("[AdvantageKit] Failed to close log file.", false);
        }

        sendPathToAdvantageScope();
    }

    @Override
    public void putTable(LogTable table) {
        if (!isOpen) {
            return;
        }

        if (autoRename) {
            updateAutoName(table);
        }

        try {
            encoder.appendInteger(timestampID, table.getTimestamp(), table.getTimestamp());

            Map<String, LogValue> newMap = table.getAll(false);
            Map<String, LogValue> oldMap = lastTable.getAll(false);

            for (Map.Entry<String, LogValue> field : newMap.entrySet()) {
                String key = field.getKey();
                LogValue value = field.getValue();
                String unit = value.unitStr;

                boolean appendData;
                if (!entryIDs.containsKey(key)) {
                    String metadata = unit == null
                            ? WPILOGConstants.ENTRY_METADATA
                            : WPILOGConstants.ENTRY_METADATA_UNITS.replace("$UNITSTR", unit);
                    int id = encoder.startEntry(key, value.getWPILOGType(), metadata, table.getTimestamp());
                    entryIDs.put(key, id);
                    entryTypes.put(key, value.type);
                    if (unit != null) {
                        entryUnits.put(key, unit);
                    }
                    appendData = true;
                } else {

                    appendData = !value.equals(oldMap.get(key));
                }

                if (appendData) {
                    int id = entryIDs.get(key);

                    if (unit != null && !unit.equals(entryUnits.get(key))) {
                        encoder.setMetadata(
                                id,
                                WPILOGConstants.ENTRY_METADATA_UNITS.replace("$UNITSTR", unit),
                                table.getTimestamp());
                        entryUnits.put(key, unit);
                    }

                    appendValue(id, value, table.getTimestamp());
                }
            }

            if (table.getTimestamp() - lastFlushTimestamp > FLUSH_PERIOD_US) {
                flush();
                lastFlushTimestamp = table.getTimestamp();
            }
        } catch (IOException e) {
            DriverStation.reportError("[AdvantageKit] Failed to write log data: " + e.getMessage(), false);
        }

        lastTable = table;
    }

    private void appendValue(int id, LogValue value, long timestamp) throws IOException {
        switch (value.type) {
            case Raw -> encoder.appendRaw(id, value.getRaw(), timestamp);
            case Boolean -> encoder.appendBoolean(id, value.getBoolean(), timestamp);
            case Integer -> encoder.appendInteger(id, value.getInteger(), timestamp);
            case Float -> encoder.appendFloat(id, value.getFloat(), timestamp);
            case Double -> encoder.appendDouble(id, value.getDouble(), timestamp);
            case String -> encoder.appendString(id, value.getString(), timestamp);
            case BooleanArray -> encoder.appendBooleanArray(id, value.getBooleanArray(), timestamp);
            case IntegerArray -> encoder.appendIntegerArray(id, value.getIntegerArray(), timestamp);
            case FloatArray -> encoder.appendFloatArray(id, value.getFloatArray(), timestamp);
            case DoubleArray -> encoder.appendDoubleArray(id, value.getDoubleArray(), timestamp);
            case StringArray -> encoder.appendStringArray(id, value.getStringArray(), timestamp);
        }
    }


    private void flush() throws IOException {
        encoder.flush();
        fileOutputStream.getFD().sync();
    }


    private void updateAutoName(LogTable table) {
        if (logDate == null) {
            boolean clockTrustworthy = (table.get("DriverStation/DSAttached", false)
                            && table.get("SystemStats/SystemTimeValid", false))
                    || RobotBase.isSimulation();

            if (clockTrustworthy) {
                double now = RobotController.getFPGATime() / 1000000.0;
                if (dsAttachedTime == null) {
                    dsAttachedTime = now;
                } else if (now - dsAttachedTime > TIMESTAMP_UPDATE_DELAY_SEC || RobotBase.isSimulation()) {
                    logDate = LocalDateTime.now();
                }
            } else {
                dsAttachedTime = null;
            }
        }

        MatchType matchType = switch (table.get("DriverStation/MatchType", 0)) {
            case 1 -> MatchType.Practice;
            case 2 -> MatchType.Qualification;
            case 3 -> MatchType.Elimination;
            default -> MatchType.None;
        };

        if (logMatchText == null && matchType != MatchType.None) {
            String prefix = switch (matchType) {
                case Practice -> "p";
                case Qualification -> "q";
                case Elimination -> "e";
                default -> "";
            };
            logMatchText = prefix + table.get("DriverStation/MatchNumber", 0);
        }

        StringBuilder newName = new StringBuilder("akit_");
        newName.append(logDate == null ? randomIdentifier : TIME_FORMATTER.format(logDate));

        String eventName = table.get("DriverStation/EventName", "").toLowerCase();
        if (!eventName.isEmpty()) {
            newName.append("_").append(eventName);
        }
        if (logMatchText != null) {
            newName.append("_").append(logMatchText);
        }
        newName.append(".wpilogxz");

        String newFilename = newName.toString();
        if (!newFilename.equals(filename)) {
            System.out.println("[AdvantageKit] Renaming log to \"" + Path.of(folder, newFilename) + "\"");
            if (new File(folder, filename).renameTo(new File(folder, newFilename))) {
                filename = newFilename;
            }
        }
    }

    private void sendPathToAdvantageScope() {
        boolean shouldOpen = switch (openBehavior) {
            case ALWAYS -> RobotBase.isSimulation();
            case AUTO -> RobotBase.isSimulation() && Logger.hasReplaySource();
            case NEVER -> false;
        };
        if (!shouldOpen) {
            return;
        }

        try {
            String fullLogPath = FileSystems.getDefault()
                    .getPath(folder, filename)
                    .normalize()
                    .toAbsolutePath()
                    .toString();
            Path tempPath = Paths.get(System.getProperty("java.io.tmpdir"), ADVANTAGE_SCOPE_FILE_NAME);
            try (PrintWriter writer = new PrintWriter(tempPath.toString(), "UTF-8")) {
                writer.println(fullLogPath);
            }
            System.out.println("[AdvantageKit] Log sent to AdvantageScope.");
        } catch (Exception e) {
            DriverStation.reportError("[AdvantageKit] Failed to send log to AdvantageScope.", false);
        }
    }
}
