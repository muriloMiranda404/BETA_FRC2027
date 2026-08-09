package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double appliedVolts = 0;
        public boolean atTop = false;
        public boolean atBottom = false;
    }

    default void processInputs(ClimberIOInputsAutoLogged inputs) {}


    default void setOutput(double percent) {}

    default void stop() {}

    default void resetEncoder(double position) {}
}
