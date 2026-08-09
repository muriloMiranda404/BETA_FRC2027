// Adapted from FRC 6328 (Mechanical Advantage) — org.littletonrobotics.frc2026.util.logging.WPILOGExtractor.
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file
// at the root directory of the original project.

package frc.frc_java9485.utils.logger.wpilogxz;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;


public class WPILOGExtractor {

    private static final String OUTPUT_SUFFIX = "_extracted.csv";

    private WPILOGExtractor() {}

    public static void main(String[] args) throws Exception {
        if (args.length < 2) {
            System.out.println("Usage: WPILOGExtractor <logfile|directory> <key1> [key2] ...");
            return;
        }

        String[] keys = Arrays.copyOfRange(args, 1, args.length);
        List<Path> logFiles = resolveLogFiles(Path.of(args[0]));

        if (logFiles.isEmpty()) {
            System.out.println("No .wpilog or .wpilogxz files found.");
            return;
        }

        for (Path logFile : logFiles) {
            System.out.printf("Processing %s...%n", logFile);
            try {
                extract(logFile, keys);
            } catch (Exception e) {
                System.out.printf("  Error: %s%n", e.getMessage());
            }
        }
    }

    private static List<Path> resolveLogFiles(Path path) throws IOException {
        if (Files.isRegularFile(path)) {
            return List.of(path);
        }
        if (Files.isDirectory(path)) {
            try (Stream<Path> entries = Files.list(path)) {
                return entries.filter(Files::isRegularFile)
                        .filter(p -> {
                            String name = p.getFileName().toString();
                            return name.endsWith(".wpilog") || name.endsWith(".wpilogxz");
                        })
                        .sorted()
                        .toList();
            }
        }
        return List.of();
    }

    private static void extract(Path logFile, String[] keys) throws Exception {
        String filename = logFile.toString();

        Iterable<DataLogRecord> reader;
        if (filename.endsWith(".wpilogxz")) {
            reader = new WPILOGXZDecoder(filename);
        } else if (filename.endsWith(".wpilog")) {
            reader = new DataLogReader(filename);
        } else {
            System.out.println("  Unsupported file format. Expected .wpilog or .wpilogxz");
            return;
        }

        Map<Integer, String> names = new HashMap<>();
        Map<Integer, String> types = new HashMap<>();

        Map<String, Map<Long, Double>> series = new LinkedHashMap<>();
        for (String key : keys) {
            series.put(key, new HashMap<>());
        }

        var iterator = reader.iterator();
        while (iterator.hasNext()) {
            DataLogRecord record;
            try {
                record = iterator.next();
            } catch (RuntimeException e) {
                break;
            }

            if (record.isControl() && record.isStart()) {
                var start = record.getStartData();
                names.put(start.entry, start.name);
                types.put(start.entry, start.type);
            } else if (!record.isControl()) {
                String name = names.get(record.getEntry());
                if (name == null) {
                    continue;
                }
                String clean = name.startsWith("/") ? name.substring(1) : name;

                Map<Long, Double> target = series.get(clean);
                if (target == null) {
                    continue;
                }

                String type = types.getOrDefault(record.getEntry(), "");
                if (type.equals("double")) {
                    target.put(record.getTimestamp(), record.getDouble());
                } else if (type.equals("float")) {
                    target.put(record.getTimestamp(), (double) record.getFloat());
                }
            }
        }

        writeCsv(filename, keys, series);
    }

    private static void writeCsv(String filename, String[] keys, Map<String, Map<Long, Double>> series)
            throws IOException {
        String primaryKey = keys[0];
        Map<Long, Double> primaryData = series.get(primaryKey);
        List<Long> sortedTimestamps = primaryData.keySet().stream().sorted().toList();

        if (sortedTimestamps.isEmpty()) {
            System.out.println("  No data found for primary key: " + primaryKey);
            return;
        }
        long firstTimestamp = sortedTimestamps.get(0);

        Map<String, InterpolationData> interpolations = new LinkedHashMap<>();
        for (int i = 1; i < keys.length; i++) {
            var sorted = series.get(keys[i]).entrySet().stream()
                    .sorted(Map.Entry.comparingByKey())
                    .toList();
            interpolations.put(keys[i], new InterpolationData(
                    sorted.stream().mapToLong(Map.Entry::getKey).toArray(),
                    sorted.stream().mapToDouble(Map.Entry::getValue).toArray()));
        }

        String outPath = filename.replaceAll("\\.[^.]+$", "") + OUTPUT_SUFFIX;
        try (PrintWriter writer = new PrintWriter(new FileWriter(outPath))) {
            writer.println("t_seconds,"
                    + Arrays.stream(keys).map(k -> k.replace("/", "_")).collect(Collectors.joining(",")));

            for (long timestamp : sortedTimestamps) {
                StringBuilder line = new StringBuilder();
                line.append(String.format("%.6f", (timestamp - firstTimestamp) / 1e6));
                line.append(String.format(",%.6f", primaryData.get(timestamp)));
                for (int i = 1; i < keys.length; i++) {
                    InterpolationData data = interpolations.get(keys[i]);
                    line.append(String.format(",%.6f", interpolate(data.times, data.values, timestamp)));
                }
                writer.println(line);
            }
        }

        System.out.printf("  Wrote %d samples to %s%n", sortedTimestamps.size(), outPath);
    }

    private record InterpolationData(long[] times, double[] values) {}


    static double interpolate(long[] times, double[] values, long at) {
        if (times.length == 0) {
            return 0.0;
        }
        if (at <= times[0]) {
            return values[0];
        }
        if (at >= times[times.length - 1]) {
            return values[values.length - 1];
        }

        int i = 0;
        while (i < times.length - 1 && times[i + 1] < at) {
            i++;
        }
        double fraction = (double) (at - times[i]) / (times[i + 1] - times[i]);
        return values[i] + fraction * (values[i + 1] - values[i]);
    }
}
