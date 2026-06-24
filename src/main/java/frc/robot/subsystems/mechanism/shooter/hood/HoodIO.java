package frc.robot.subsystems.mechanism.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public static class HoodIOInputs {
        public double hoodSetpoit = 0;
        public boolean atSetpoint = false;
        public boolean atLimit = false;
        public boolean atHome = false;
        public double hoodAngleDeg = 0;
    }

    default void processInputs(HoodIOInputsAutoLogged inputs){};

    default void setHoodFromSetpoint(double position){};

    default void resetHood(){};

    default void off(){};

    default void returnHoodToHome(){};
}
