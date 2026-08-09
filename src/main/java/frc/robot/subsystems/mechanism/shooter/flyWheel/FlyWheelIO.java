package frc.robot.subsystems.mechanism.shooter.flyWheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlyWheelIO {

    @AutoLog
    public static class FlyWheelIOInputs{
        public double averageSpeed = 0;
        public double speedSetpoint = 0;
        public boolean atSetpoint = false;
        public boolean isShooting = false;
    }

    default void processInputs(FlyWheelIOInputsAutoLogged inputs){};

    default void setFlyWheelSpeed(double speed){};

    default void stop(){};
}
