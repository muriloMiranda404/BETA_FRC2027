// Copyright (c) 2025-2026 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Adapted for RebuiltHyobots2026: package renamed, plus per-loop accounting so something can
// actually watch the numbers instead of only recording them.

package frc.frc_java9485.utils.logger;

import java.util.LinkedHashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.frc_java9485.constants.utils.LoggerConstants;


public class LoggedTracer {

    private LoggedTracer() {}


    private static double phaseStartTime = -1.0;


    private static double loopStartTime = -1.0;

    private static final Map<String, Double> currentEpochs = new LinkedHashMap<>();

    private static double lastLoopMs = 0.0;
    private static String worstEpochName = "";
    private static double worstEpochMs = 0.0;

    private static int overrunCount = 0;
    private static double lastOverrunTimestamp = Double.NEGATIVE_INFINITY;


    public static void reset() {
        double now = Timer.getFPGATimestamp();


        if (loopStartTime >= 0.0) {
            finishLoop(now);
        }

        loopStartTime = now;
        phaseStartTime = now;
        currentEpochs.clear();
    }


    public static void record(String epochName) {
        double now = Timer.getFPGATimestamp();
        double elapsedMs = (now - phaseStartTime) * 1000.0;

        Logger.recordOutput("LoggedTracer/" + epochName + "MS", elapsedMs);


        currentEpochs.merge(epochName, elapsedMs, Double::sum);
        phaseStartTime = now;
    }

    private static void finishLoop(double now) {
        lastLoopMs = (now - loopStartTime) * 1000.0;

        worstEpochName = "";
        worstEpochMs = 0.0;
        for (Map.Entry<String, Double> epoch : currentEpochs.entrySet()) {
            if (epoch.getValue() > worstEpochMs) {
                worstEpochMs = epoch.getValue();
                worstEpochName = epoch.getKey();
            }
        }

        if (lastLoopMs > LoggerConstants.LOOP_OVERRUN_THRESHOLD_MS) {
            overrunCount++;
            lastOverrunTimestamp = now;
        }

        Logger.recordOutput("LoggedTracer/LoopMS", lastLoopMs);
        Logger.recordOutput("LoggedTracer/WorstEpoch", worstEpochName);
        Logger.recordOutput("LoggedTracer/WorstEpochMS", worstEpochMs);
        Logger.recordOutput("LoggedTracer/Overruns", overrunCount);
    }


    public static double getLastLoopMs() {
        return lastLoopMs;
    }


    public static String getWorstEpochName() {
        return worstEpochName;
    }


    public static double getWorstEpochMs() {
        return worstEpochMs;
    }


    public static int getOverrunCount() {
        return overrunCount;
    }


    public static boolean isLastLoopOverrun() {
        return lastLoopMs > LoggerConstants.LOOP_OVERRUN_THRESHOLD_MS;
    }


    public static boolean hasRecentOverrun(double windowSeconds) {
        return Timer.getFPGATimestamp() - lastOverrunTimestamp < windowSeconds;
    }


    public static void resetStatistics() {
        phaseStartTime = -1.0;
        loopStartTime = -1.0;
        currentEpochs.clear();
        lastLoopMs = 0.0;
        worstEpochName = "";
        worstEpochMs = 0.0;
        overrunCount = 0;
        lastOverrunTimestamp = Double.NEGATIVE_INFINITY;
    }
}
