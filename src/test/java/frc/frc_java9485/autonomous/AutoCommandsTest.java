package frc.frc_java9485.autonomous;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


class AutoCommandsTest {

    private static final double LOOP_PERIOD = 0.02;

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


        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();

        CommandScheduler.getInstance().cancelAll();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }


    private static double runUntilFinished(Command command, double limitSeconds) {
        command.schedule();
        assertTrue(command.isScheduled(), "command was not scheduled; is the robot enabled?");

        double elapsed = 0.0;
        while (command.isScheduled() && elapsed < limitSeconds) {
            SimHooks.stepTiming(LOOP_PERIOD);
            CommandScheduler.getInstance().run();
            elapsed += LOOP_PERIOD;
        }
        return elapsed;
    }

    @Test
    void neverReadyStillFinishesAtTheBackstop() {
        Command command = AutoCommands.feedWhenReady(() -> false, "Test");

        double elapsed = runUntilFinished(command, 10.0);
        double expected = AutoCommands.getReadyTimeoutSeconds() + AutoCommands.getFeedWindowSeconds();

        assertFalse(command.isScheduled(), "a shot that never verifies must still end");
        assertTrue(Math.abs(elapsed - expected) < 0.15,
                "expected ~" + expected + "s (ready timeout + feed window), got " + elapsed);
    }

    @Test
    void readyImmediatelyFinishesAfterOnlyTheFeedWindow() {
        Command command = AutoCommands.feedWhenReady(() -> true, "Test");

        double elapsed = runUntilFinished(command, 10.0);

        assertTrue(elapsed < AutoCommands.getFeedWindowSeconds() + 0.15,
                "an already-verified shot should only take the feed window, took " + elapsed);
    }


    @Test
    void feedWindowStartsAfterReadinessRatherThanBeingConsumedByIt() {
        double readyDelay = 0.6;
        double[] elapsed = {0.0};

        Command command = AutoCommands.feedWhenReady(() -> elapsed[0] >= readyDelay, "Test");
        command.schedule();

        while (command.isScheduled() && elapsed[0] < 10.0) {
            SimHooks.stepTiming(LOOP_PERIOD);
            CommandScheduler.getInstance().run();
            elapsed[0] += LOOP_PERIOD;
        }

        double expected = readyDelay + AutoCommands.getFeedWindowSeconds();
        assertTrue(Math.abs(elapsed[0] - expected) < 0.15,
                "feed window should start after readiness: expected ~" + expected + "s, got " + elapsed[0]);


        assertTrue(elapsed[0] > AutoCommands.getFeedWindowSeconds() + 0.3,
                "a late-verifying shot must take longer overall, not the same");
    }


    @Test
    void readinessAfterTheBackstopDoesNotExtendTheCommand() {
        double[] elapsed = {0.0};

        Command command = AutoCommands.feedWhenReady(
                () -> elapsed[0] >= AutoCommands.getReadyTimeoutSeconds() + 1.0, "Test");
        command.schedule();

        while (command.isScheduled() && elapsed[0] < 10.0) {
            SimHooks.stepTiming(LOOP_PERIOD);
            CommandScheduler.getInstance().run();
            elapsed[0] += LOOP_PERIOD;
        }

        double bound = AutoCommands.getReadyTimeoutSeconds() + AutoCommands.getFeedWindowSeconds();
        assertTrue(elapsed[0] < bound + 0.15,
                "command must stay within its backstop, took " + elapsed[0]);
    }
}
