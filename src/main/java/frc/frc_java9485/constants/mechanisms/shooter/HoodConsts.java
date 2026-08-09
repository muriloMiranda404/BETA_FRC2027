package frc.frc_java9485.constants.mechanisms.shooter;

public class HoodConsts {
    public static final class Motor{
        public static final int HOOD_MOTOR_ID = 15;
    }

    public static final class Configs{
        public static final int HOOD_CURRENT_LIMIT = 30;
    }

    public static final class Setpoint{
        public static final double MAX_POSITION = 3.5;
        public static final double MIN_POSITION = 0.0;
    }


    public static final class PID{
        public static final double Kp = 8.0;
        public static final double Ki = 0.00;
        public static final double Kd = 0.10;


        public static final double Ks = 0.15;


        public static final double Kg = 0.20;
    }




}
