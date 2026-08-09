package frc.frc_java9485.utils.logger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.frc_java9485.constants.utils.LoggerConstants;


class LoggedTracerTest {

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @AfterAll
    static void shutdownHAL() {
        HAL.shutdown();
    }

    @BeforeEach
    void setUp() {
        SimHooks.pauseTiming();
        LoggedTracer.resetStatistics();
    }

    @AfterEach
    void tearDown() {
        LoggedTracer.resetStatistics();
        SimHooks.resumeTiming();
    }


    private static void runLoop(double... phaseMillis) {
        LoggedTracer.reset();
        for (int i = 0; i < phaseMillis.length; i++) {
            SimHooks.stepTiming(phaseMillis[i] / 1000.0);
            LoggedTracer.record("Phase" + i);
        }
    }

    @Test
    void healthyLoopIsNotFlaggedAsOverrun() {
        runLoop(2.0, 3.0, 4.0);

        LoggedTracer.reset();

        assertEquals(9.0, LoggedTracer.getLastLoopMs(), 0.5);
        assertFalse(LoggedTracer.isLastLoopOverrun());
        assertEquals(0, LoggedTracer.getOverrunCount());
    }

    @Test
    void slowLoopIsCountedAsAnOverrun() {
        runLoop(5.0, 30.0);
        LoggedTracer.reset();

        assertTrue(LoggedTracer.getLastLoopMs() > LoggerConstants.LOOP_OVERRUN_THRESHOLD_MS);
        assertTrue(LoggedTracer.isLastLoopOverrun());
        assertEquals(1, LoggedTracer.getOverrunCount());
    }


    @Test
    void slowestPhaseIsIdentified() {
        runLoop(2.0, 30.0, 3.0);
        LoggedTracer.reset();

        assertEquals("Phase1", LoggedTracer.getWorstEpochName());
        assertEquals(30.0, LoggedTracer.getWorstEpochMs(), 1.0);
    }


    @Test
    void repeatedPhaseAccumulates() {
        LoggedTracer.reset();
        SimHooks.stepTiming(0.006);
        LoggedTracer.record("Vision");
        SimHooks.stepTiming(0.002);
        LoggedTracer.record("Other");
        SimHooks.stepTiming(0.007);
        LoggedTracer.record("Vision");
        LoggedTracer.reset();

        assertEquals("Vision", LoggedTracer.getWorstEpochName());
        assertEquals(13.0, LoggedTracer.getWorstEpochMs(), 1.0);
    }

    @Test
    void overrunsAccumulateAcrossLoops() {
        for (int i = 0; i < 3; i++) {
            runLoop(30.0);
        }
        LoggedTracer.reset();

        assertEquals(3, LoggedTracer.getOverrunCount());
    }


    @Test
    void recentOverrunWindowExpires() {
        runLoop(30.0);
        LoggedTracer.reset();

        assertTrue(LoggedTracer.hasRecentOverrun(2.0), "should report the overrun that just happened");

        SimHooks.stepTiming(3.0);
        assertFalse(LoggedTracer.hasRecentOverrun(2.0), "should clear once the window has passed");
    }

    @Test
    void noOverrunEverMeansNoRecentOverrun() {
        runLoop(2.0, 2.0);
        LoggedTracer.reset();

        assertFalse(LoggedTracer.hasRecentOverrun(2.0));
    }


    @Test
    void firstResetDoesNotFabricateALoop() {
        LoggedTracer.reset();

        assertEquals(0.0, LoggedTracer.getLastLoopMs(), 1e-9);
        assertEquals(0, LoggedTracer.getOverrunCount());
        assertEquals("", LoggedTracer.getWorstEpochName());
    }
}
