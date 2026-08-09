package frc.robot.subsystems.mechanism.shooter.turret;

import org.littletonrobotics.junction.AutoLog;


public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs {

        public double turretAngle = 0;
        public boolean atSetpoint = false;
        public boolean isBrakeMode = false;
        public boolean atMax = false;
        public boolean atMin = false;

        public double turretSetpoint = 0;
    }

    default void processInputs(TurretIOInputsAutoLogged inputs){};


    default void setTurretPosition(double degrees){};

    default void lockTurret(){};

    default void reset(){};

    default void stop(){};

    boolean atSetpoint();


    double getTurretPosition();

}
