package frc.frc_java9485.constants.robot;


public class CanIds {

    private CanIds() {}


    public static final class Swerve {
        public static final int FRONT_LEFT_DRIVE = 4;
        public static final int FRONT_LEFT_ANGLE = 3;
        public static final int FRONT_RIGHT_DRIVE = 5;
        public static final int FRONT_RIGHT_ANGLE = 6;
        public static final int BACK_LEFT_DRIVE = 1;
        public static final int BACK_LEFT_ANGLE = 2;
        public static final int BACK_RIGHT_DRIVE = 7;
        public static final int BACK_RIGHT_ANGLE = 8;
    }


    public static final class Sensors {
        public static final int CANCODER_FRONT_LEFT = 10;
        public static final int CANCODER_FRONT_RIGHT = 11;
        public static final int CANCODER_BACK_LEFT = 13;
        public static final int CANCODER_BACK_RIGHT = 12;
        public static final int PIGEON2 = 9;
    }


    public static final class Mechanisms {
        public static final int INTAKE_ROLLERS = 9;
        public static final int INTAKE_PIVOT = 10;
        public static final int FLYWHEEL_RIGHT = 11;
        public static final int FLYWHEEL_LEFT = 12;
        public static final int TURRET = 13;
        public static final int TURRET_INDEXER = 14;
        public static final int HOOD = 15;
        public static final int INDEX = 16;
        public static final int CONVEYOR = 17;
        public static final int CLIMBER = 20;
    }
}
