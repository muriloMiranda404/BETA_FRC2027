package frc.frc_java9485.constants.mechanisms.shooter;

public class FlyWheelConsts {
    public static final class Motors {
        public static final int RIGHT_SHOOTER = 11;
        public static final int LEFT_SHOOTER = 12;

        public static final int INDEXER = 14;
    }

    public static final class PID{
        public static final double Kp = 2.3;
        public static final double Ki = 1.0;
        public static final double Kd = 0.1;
    }

    public static final class Config {
        public static final int SHOOTER_CURRENT_LIMIT = 40;

        public static final int INDEXER_CURRENT_LIMIT = 30;
    }
}
