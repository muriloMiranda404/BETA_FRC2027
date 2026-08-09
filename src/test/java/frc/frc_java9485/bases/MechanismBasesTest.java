package frc.frc_java9485.bases;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;


class MechanismBasesTest {


    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @AfterAll
    static void shutdownHAL() {
        HAL.shutdown();
    }

    private enum Wanted { ON, OFF }

    private enum State { ON, OFF }


    private static class FakeInputs implements LoggableInputs {
        double measured = 0.0;
        boolean atHome = false;

        @Override
        public void toLog(LogTable table) {
            table.put("Measured", measured);
            table.put("AtHome", atHome);
        }

        @Override
        public void fromLog(LogTable table) {
            measured = table.get("Measured", measured);
            atHome = table.get("AtHome", atHome);
        }
    }

    private static class TestServo extends ServoMechanism<Wanted, State, FakeInputs> {
        int homeCount = 0;
        int appliedCount = 0;
        double lastApplied = Double.NaN;

        TestServo() {
            super("TestServo", new FakeInputs(), Wanted.OFF, State.OFF, -10.0, 10.0, 0.5);
        }

        void setMeasured(double value) {
            inputs.measured = value;
        }

        void setAtHomeSensor(boolean value) {
            inputs.atHome = value;
        }

        @Override
        public double getMeasuredPosition() {
            return inputs.measured;
        }

        @Override
        protected void readInputs(FakeInputs inputs) {}

        @Override
        protected boolean atHomeSensor() {
            return inputs.atHome;
        }

        @Override
        protected void onReachedHome() {
            homeCount++;
        }

        @Override
        protected State handleTransition(Wanted wanted) {
            return wanted == Wanted.ON ? State.ON : State.OFF;
        }

        @Override
        protected void applyState(State state, boolean stateChanged) {
            if (stateChanged) {
                appliedCount++;
            }
            lastApplied = getSetpoint();
        }
    }

    private static class TestFlywheel extends FlywheelMechanism<Wanted, State, FakeInputs> {
        TestFlywheel(double debounceSeconds) {
            super("TestFlywheel", new FakeInputs(), Wanted.OFF, State.OFF, 50.0, debounceSeconds);
        }

        void setMeasured(double rpm) {
            inputs.measured = rpm;
        }

        @Override
        public double getMeasuredRPM() {
            return inputs.measured;
        }

        @Override
        protected void readInputs(FakeInputs inputs) {}

        @Override
        protected State handleTransition(Wanted wanted) {
            return wanted == Wanted.ON ? State.ON : State.OFF;
        }

        @Override
        protected void applyState(State state, boolean stateChanged) {}
    }



    @Test
    void servo_clampsSetpointToMechanicalTravel() {
        TestServo servo = new TestServo();

        servo.setSetpoint(50.0);
        assertEquals(10.0, servo.getSetpoint(), 1e-9);

        servo.setSetpoint(-50.0);
        assertEquals(-10.0, servo.getSetpoint(), 1e-9);

        servo.setSetpoint(3.0);
        assertEquals(3.0, servo.getSetpoint(), 1e-9);
    }

    @Test
    void servo_atSetpointRespectsTolerance() {
        TestServo servo = new TestServo();
        servo.setSetpoint(5.0);

        servo.setMeasured(5.4);
        assertTrue(servo.atSetpoint());

        servo.setMeasured(5.6);
        assertFalse(servo.atSetpoint());
    }

    @Test
    void servo_homesOnRisingEdgeOnly() {
        TestServo servo = new TestServo();

        servo.setAtHomeSensor(true);
        servo.update();
        servo.update();
        assertEquals(1, servo.homeCount, "resting on the home sensor must not re-home every loop");

        servo.setAtHomeSensor(false);
        servo.update();
        servo.setAtHomeSensor(true);
        servo.update();
        assertEquals(2, servo.homeCount, "leaving and returning must home again");
    }

    @Test
    void stateMachine_reportsStateChangeOnlyOnFirstLoop() {
        TestServo servo = new TestServo();

        servo.setWantedState(Wanted.ON);
        servo.update();
        assertEquals(State.ON, servo.getCurrentState());
        assertEquals(1, servo.appliedCount);

        servo.update();
        assertEquals(1, servo.appliedCount, "staying in a state is not a transition");

        servo.setWantedState(Wanted.OFF);
        servo.update();
        assertEquals(2, servo.appliedCount);
        assertEquals(State.ON, servo.getLastState());
    }



    @Test
    void flywheel_zeroSetpointIsNeverSpunUp() {
        TestFlywheel flywheel = new TestFlywheel(0.0);
        flywheel.setSetpointRPM(0.0);
        flywheel.setMeasured(0.0);

        assertFalse(flywheel.spunUp(), "a stopped flywheel must not advertise itself as ready");
    }

    @Test
    void flywheel_spunUpRespectsTolerance() {
        TestFlywheel flywheel = new TestFlywheel(0.0);
        flywheel.setSetpointRPM(3000.0);

        flywheel.setMeasured(2960.0);
        assertTrue(flywheel.spunUp());

        flywheel.setMeasured(2900.0);
        assertFalse(flywheel.spunUp());
    }

    @Test
    void flywheel_atSetpointIsDebouncedAndRefreshedOncePerUpdate() {

        TestFlywheel flywheel = new TestFlywheel(0.06);
        flywheel.setSetpointRPM(3000.0);
        flywheel.setMeasured(3000.0);

        flywheel.update();
        assertFalse(flywheel.atSetpoint(), "readiness must not latch on the first good sample");


        assertFalse(flywheel.atSetpoint());
        assertFalse(flywheel.atSetpoint());
    }
}
