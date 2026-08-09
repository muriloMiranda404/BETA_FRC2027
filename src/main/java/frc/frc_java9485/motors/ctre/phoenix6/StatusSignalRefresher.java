package frc.frc_java9485.motors.ctre.phoenix6;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.StatusSignal;


public class StatusSignalRefresher {

    private static StatusSignalRefresher instance;

    public static StatusSignalRefresher getInstance() {
        if (instance == null) {
            instance = new StatusSignalRefresher();
        }
        return instance;
    }

    private final Map<Integer, List<StatusSignal<?>>> statusSignalMap = new HashMap<>();
    private final Map<Integer, String> calculatedStatusSignalsPerCycle = new HashMap<>();
    private final Map<String, StatusSignal<?>[]> calculatedStatusSignalsArrays = new HashMap<>();
    private int cycleCount;

    private StatusSignalRefresher() {}


    public void addStatusSignals(StatusSignal<?>... signals) {
        addStatusSignals(0, signals);
    }


    public void addStatusSignals(int delayLoopCount, StatusSignal<?>... signals) {
        List<StatusSignal<?>> list = statusSignalMap.computeIfAbsent(delayLoopCount, k -> new ArrayList<>());
        Collections.addAll(list, signals);
    }


    public void finalizeStatusSignals() {
        calculatedStatusSignalsPerCycle.clear();
        calculatedStatusSignalsArrays.clear();

        Integer[] statusSignalDelayCounts = statusSignalMap.keySet().stream()
                .filter(a -> a != 0)
                .sorted()
                .toArray(Integer[]::new);

        if (statusSignalDelayCounts.length == 0) {
            calculatedStatusSignalsPerCycle.put(0, "0");
            calculatedStatusSignalsArrays.put(
                    "0", statusSignalMap.getOrDefault(0, List.of()).toArray(new StatusSignal<?>[0]));
            return;
        }


        Integer[] countedValues = statusSignalDelayCounts.clone();
        int largestNumber = countedValues[countedValues.length - 1];
        while (!Arrays.stream(countedValues).allMatch(a -> a.equals(countedValues[0]))) {
            for (int i = 0; i < countedValues.length; i++) {
                while (largestNumber > countedValues[i]) {
                    countedValues[i] += statusSignalDelayCounts[i];
                }
                largestNumber = countedValues[i];
            }
        }


        List<StatusSignal<?>> toRefresh = new ArrayList<>();
        for (int i = 0; i < largestNumber; i++) {
            StringBuilder sb = new StringBuilder();
            for (Integer key : statusSignalMap.keySet()) {
                if (i % (key + 1) == 0) {
                    toRefresh.addAll(statusSignalMap.get(key));
                    if (sb.length() > 0) {
                        sb.append(",");
                    }
                    sb.append(key);
                }
            }
            String keyPattern = sb.toString();
            calculatedStatusSignalsPerCycle.put(i, keyPattern);
            calculatedStatusSignalsArrays.computeIfAbsent(
                    keyPattern, k -> toRefresh.toArray(new StatusSignal<?>[0]));
            toRefresh.clear();
        }
    }


    public void refreshStatusSignals() {
        int calculatedCount = calculatedStatusSignalsPerCycle.size();
        if (calculatedCount == 0) {
            return;
        }

        StatusSignal<?>[] toRefresh;
        if (calculatedCount <= 1) {
            toRefresh = calculatedStatusSignalsArrays.get("0");
        } else {
            String key = calculatedStatusSignalsPerCycle.get(cycleCount % (calculatedCount - 1));
            toRefresh = calculatedStatusSignalsArrays.get(key);
        }

        if (toRefresh != null && toRefresh.length > 0) {
            StatusSignal.refreshAll(toRefresh);
        }
        cycleCount++;
    }
}
