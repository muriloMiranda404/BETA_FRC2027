package frc.robot.subsystems.mechanism;

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
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIOInputsAutoLogged;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIOSim;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelSubsystem;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIOSim;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIOInputsAutoLogged;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIOSim;


class MechanismSimulationTest {

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
    void pauseClock() {

        SimHooks.pauseTiming();
    }

    @AfterEach
    void resumeClock() {
        SimHooks.resumeTiming();
    }

    private static void step() {
        SimHooks.stepTiming(LOOP_PERIOD);
    }



    @Test
    void flywheelDoesNotReachSetpointInstantly() {
        FlyWheelIOSim io = new FlyWheelIOSim();
        FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();

        io.setFlyWheelSpeed(3000.0);
        step();
        io.processInputs(inputs);

        assertTrue(inputs.averageSpeed < 3000.0,
                "a real flywheel cannot be at speed after one loop, got " + inputs.averageSpeed);
        assertFalse(inputs.atSetpoint);
    }

    @Test
    void flywheelSpinsUpMonotonicallyAndSettles() {
        FlyWheelIOSim io = new FlyWheelIOSim();
        FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();

        io.setFlyWheelSpeed(3000.0);

        double previous = -1.0;
        boolean reached = false;
        for (int i = 0; i < 250; i++) {
            step();
            io.processInputs(inputs);

            assertTrue(inputs.averageSpeed >= previous - 1.0,
                    "spin-up should not go backwards: " + previous + " -> " + inputs.averageSpeed);
            previous = inputs.averageSpeed;

            if (inputs.atSetpoint) {
                reached = true;
                break;
            }
        }

        assertTrue(reached, "flywheel never reached 3000 RPM, stopped at " + previous);
    }


    @Test
    void flywheelCoastsDownWithoutReversing() {
        FlyWheelIOSim io = new FlyWheelIOSim();
        FlyWheelIOInputsAutoLogged inputs = new FlyWheelIOInputsAutoLogged();

        io.setFlyWheelSpeed(3000.0);
        for (int i = 0; i < 250; i++) {
            step();
            io.processInputs(inputs);
        }

        io.stop();
        for (int i = 0; i < 100; i++) {
            step();
            io.processInputs(inputs);
            assertTrue(inputs.averageSpeed >= -1.0, "flywheel reversed while stopping");
        }
        assertFalse(inputs.atSetpoint, "a stopped flywheel must never report ready");
    }


    @Test
    void flywheelSubsystemReadinessLagsTheCommand() {
        FlyWheelSubsystem flywheel = new FlyWheelSubsystem(new FlyWheelIOSim());

        flywheel.setShootingRPM(3000.0);
        flywheel.setWantedState(FlyWheelSubsystem.WantedState.SHOOTING);

        step();
        flywheel.update();
        assertFalse(flywheel.spunUp(), "the flywheel cannot be up to speed after one loop");

        boolean spunUp = false;
        for (int i = 0; i < 300 && !spunUp; i++) {
            step();
            flywheel.update();
            spunUp = flywheel.spunUp();
        }
        assertTrue(spunUp, "flywheel never reached its commanded speed");
    }


    @Test
    void flywheelSubsystemDropsReadinessWhenTurnedOff() {
        FlyWheelSubsystem flywheel = new FlyWheelSubsystem(new FlyWheelIOSim());

        flywheel.setShootingRPM(3000.0);
        flywheel.setWantedState(FlyWheelSubsystem.WantedState.SHOOTING);
        for (int i = 0; i < 300 && !flywheel.spunUp(); i++) {
            step();
            flywheel.update();
        }
        assertTrue(flywheel.spunUp());

        flywheel.setWantedState(FlyWheelSubsystem.WantedState.OFF);
        step();
        flywheel.update();

        assertFalse(flywheel.spunUp(), "a stopped flywheel must not advertise itself as ready");
    }



    @Test
    void turretConvergesOnItsSetpoint() {
        TurretIOSim io = new TurretIOSim();
        TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

        io.setTurretPosition(45.0);
        for (int i = 0; i < 400; i++) {
            step();
            io.processInputs(inputs);
        }

        assertEquals(45.0, inputs.turretAngle, TurretConsts.Setpoint.TOLERANCE_DEG * 2.0);
        assertTrue(inputs.atSetpoint);
    }

    @Test
    void turretDoesNotTeleport() {
        TurretIOSim io = new TurretIOSim();
        TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

        io.setTurretPosition(80.0);
        step();
        io.processInputs(inputs);

        assertTrue(inputs.turretAngle < 20.0,
                "turret moved " + inputs.turretAngle + " deg in one loop, which is not physical");
    }


    @Test
    void turretStopsAtItsHardStop() {
        TurretIOSim io = new TurretIOSim();
        TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();


        io.setTurretPosition(1000.0);
        for (int i = 0; i < 600; i++) {
            step();
            io.processInputs(inputs);
        }

        assertTrue(inputs.turretAngle <= TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG + 0.5,
                "turret passed its hard stop: " + inputs.turretAngle);
        assertTrue(inputs.atMax);
    }



    @Test
    void hoodStartsHomeAndReportsTheHomeSensor() {
        HoodIOSim io = new HoodIOSim();
        HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

        step();
        io.processInputs(inputs);

        assertTrue(inputs.atHome, "hood should start on its home sensor");
    }

    @Test
    void hoodHoldsPositionAgainstGravity() {
        HoodIOSim io = new HoodIOSim();
        HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

        io.setHoodFromSetpoint(2.0);
        for (int i = 0; i < 400; i++) {
            step();
            io.processInputs(inputs);
        }

        assertEquals(2.0, inputs.hoodPosition, 0.15);
        assertFalse(inputs.atHome, "hood should have left the home sensor");
    }
}
