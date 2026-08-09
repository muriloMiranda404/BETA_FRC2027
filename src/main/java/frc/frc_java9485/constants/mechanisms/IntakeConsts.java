package frc.frc_java9485.constants.mechanisms;

public class IntakeConsts {
    public static final class Motors{
      public static final int PIVOT_ID = 10;
      public static final int CATCH_BALL_ID = 9;
    }

    public static final class Encoder{
      public static final boolean ENCODER_INVERTED = true;
      public static final int ENCODER_CHANNEL = 9;
    }


    public static final class Setpoint{
      public static final double SETPOINT_UP = 250.00;
      public static final double SETPOINT_MIDDLE = 200;
      public static final double SETPOINT_DOWN = 16;

      public static final double STOPPED_FUEL_SPEED = 0;
      public static final double COLLECT_FUEL_SPEED = 0.7;
    }


    public static final class PID{
      public static final double Kp = 8.0;
      public static final double Ki = 0.00;
      public static final double Kd = 0.10;
    }


    public static final class FeedForward{
      public static final double Kg = 0.35;

      public static final double Ks = 0.20;
    }
}
