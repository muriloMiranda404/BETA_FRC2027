// Adapted from FRC 6328 (Mechanical Advantage) — org.littletonrobotics.frc2026.util.logging.WPILOGXZReader.
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file
// at the root directory of the original project.

package frc.frc_java9485.utils.logger.wpilogxz;

import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogReplaySource;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;
import org.littletonrobotics.junction.LogTable.LoggableType;

import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;


public class WPILOGXZReader implements LogReplaySource {

    private final String filename;

    private WPILOGXZDecoder decoder;
    private Iterator<DataLogRecord> iterator;
    private boolean isValid;

    private Long timestamp;
    private Map<Integer, String> entryIDs;
    private Map<Integer, LoggableType> entryTypes;
    private Map<Integer, String> entryCustomTypes;
    private Map<Integer, String> entryUnits;

    public WPILOGXZReader(String filename) {
        this.filename = filename;
    }

    @Override
    public void start() {
        try {
            double startTime = Timer.getFPGATimestamp();
            decoder = new WPILOGXZDecoder(filename);
            double endTime = Timer.getFPGATimestamp();
            System.out.printf(
                    "[AdvantageKit] Replay log file decompressed in %.1f seconds.%n", endTime - startTime);
        } catch (IOException e) {
            DriverStation.reportError(
                    "[AdvantageKit] Failed to open replay log file: " + e.getMessage(), false);
            isValid = false;
            return;
        }

        if (!decoder.isValid()) {
            DriverStation.reportError("[AdvantageKit] The replay log is not a valid WPILOG file.", false);
            isValid = false;
        } else if (!decoder.getExtraHeader().equals(WPILOGConstants.EXTRA_HEADER)) {
            DriverStation.reportError("[AdvantageKit] The replay log was not produced by AdvantageKit.", true);
            isValid = false;
        } else {
            isValid = true;
        }

        iterator = decoder.iterator();
        timestamp = null;
        entryIDs = new HashMap<>();
        entryTypes = new HashMap<>();
        entryCustomTypes = new HashMap<>();
        entryUnits = new HashMap<>();
    }

    @Override
    public boolean updateTable(LogTable table) {
        if (!isValid) {
            return false;
        }


        if (timestamp != null) {
            table.setTimestamp(timestamp);
        }

        boolean readError = false;
        while (iterator.hasNext()) {
            DataLogRecord record;
            try {
                record = iterator.next();
            } catch (RuntimeException e) {
                readError = true;
                break;
            }

            if (record.isControl()) {
                handleControlRecord(record);
                continue;
            }

            String entry = entryIDs.get(record.getEntry());
            if (entry == null) {
                continue;
            }

            if (entry.equals(LogDataReceiver.timestampKey)) {
                boolean firstTimestamp = timestamp == null;
                timestamp = record.getInteger();
                if (firstTimestamp) {
                    table.setTimestamp(timestamp);
                } else {
                    break;
                }
            } else if (timestamp != null && record.getTimestamp() == timestamp) {
                putValue(table, record, entry.substring(1));
            }
        }

        return iterator.hasNext() && !readError;
    }

    private void handleControlRecord(DataLogRecord record) {
        if (record.isStart()) {
            var start = record.getStartData();
            entryIDs.put(start.entry, start.name);

            LoggableType loggableType = LoggableType.fromWPILOGType(start.type);
            entryTypes.put(start.entry, loggableType);
            if ((loggableType == LoggableType.Raw && !start.type.equals("raw")) || start.type.equals("json")) {
                entryCustomTypes.put(start.entry, start.type);
            }

            String unit = parseUnit(start.metadata);
            if (unit != null) {
                entryUnits.put(start.entry, unit);
            }
        } else if (record.isSetMetadata()) {
            var metadata = record.getSetMetadataData();
            String unit = parseUnit(metadata.metadata);
            if (unit != null) {
                entryUnits.put(metadata.entry, unit);
            } else {
                entryUnits.remove(metadata.entry);
            }
        }
    }

    private void putValue(LogTable table, DataLogRecord record, String entry) {

        if (entry.startsWith("ReplayOutputs")) {
            return;
        }

        String customType = entryCustomTypes.get(record.getEntry());
        String unit = entryUnits.get(record.getEntry());

        switch (entryTypes.get(record.getEntry())) {
            case Raw -> table.put(entry, new LogValue(record.getRaw(), customType));
            case Boolean -> table.put(entry, new LogValue(record.getBoolean(), customType));
            case Integer -> table.put(entry, new LogValue(record.getInteger(), customType));
            case Float -> table.put(entry, unit != null
                    ? new LogValue(record.getFloat(), customType, unit)
                    : new LogValue(record.getFloat(), customType));
            case Double -> table.put(entry, unit != null
                    ? new LogValue(record.getDouble(), customType, unit)
                    : new LogValue(record.getDouble(), customType));
            case String -> table.put(entry, new LogValue(record.getString(), customType));
            case BooleanArray -> table.put(entry, new LogValue(record.getBooleanArray(), customType));
            case IntegerArray -> table.put(entry, new LogValue(record.getIntegerArray(), customType));
            case FloatArray -> table.put(entry, new LogValue(record.getFloatArray(), customType));
            case DoubleArray -> table.put(entry, new LogValue(record.getDoubleArray(), customType));
            case StringArray -> table.put(entry, new LogValue(record.getStringArray(), customType));
        }
    }


    private static String parseUnit(String metadata) {
        if (metadata == null || !metadata.contains("\"unit\":\"")) {
            return null;
        }
        int start = metadata.indexOf("\"unit\":\"") + 8;
        int end = metadata.indexOf("\"", start);
        return end == -1 ? null : metadata.substring(start, end);
    }
}
