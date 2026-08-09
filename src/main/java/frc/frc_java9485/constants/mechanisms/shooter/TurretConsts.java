package frc.frc_java9485.constants.mechanisms.shooter;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TurretConsts {

    public static final class Motors{
        public static final int TURRET_MOTOR = 13;
    }


    public static final class ShotModel {
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM = new InterpolatingDoubleTreeMap();


        public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_POSITION = new InterpolatingDoubleTreeMap();


        public static final InterpolatingDoubleTreeMap DISTANCE_TO_TOF = new InterpolatingDoubleTreeMap();

        static {
            DISTANCE_TO_RPM.put(1.0, 2200.0);
            DISTANCE_TO_RPM.put(2.5, 2500.0);
            DISTANCE_TO_RPM.put(4.0, 2900.0);
            DISTANCE_TO_RPM.put(6.0, 3400.0);

            DISTANCE_TO_HOOD_POSITION.put(1.0, 0.5);
            DISTANCE_TO_HOOD_POSITION.put(2.5, 1.5);
            DISTANCE_TO_HOOD_POSITION.put(4.0, 2.5);
            DISTANCE_TO_HOOD_POSITION.put(6.0, 3.5);


            DISTANCE_TO_TOF.put(1.0, 0.35);
            DISTANCE_TO_TOF.put(2.5, 0.55);
            DISTANCE_TO_TOF.put(4.0, 0.80);
            DISTANCE_TO_TOF.put(6.0, 1.10);
        }
    }


    public static final class MotionComp {

        public static final double MECHANISM_LATENCY_SEC = 0.05;


        public static final double SHOOTER_FORWARD_OFFSET_M = 0.0;


        public static final int SMOOTHING_WINDOW_LOOPS = 5;
    }


    public static final class PID {
        public static final double Kp = 30.0;
        public static final double Ki = 0.00;
        public static final double Kd = 0.50;


        public static final double Ks = 0.20;
    }

    public static final class Config{
        public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
        new Transform3d(new Translation3d(Inches.zero(), Inches.of(7), Inches.of(17.5)), Rotation3d.kZero);

        public static final int TURRET_CURRENT_LIMIT = 30;


        public static final double TURRET_REDUCTION = 66.8;
    }

    public static final class Setpoint{

        public static final double MAX_TURN_POSITION = 17.78;

        public static final double MIN_TURN_POSITION = -19.69;


        public static final double MOTOR_ROTATIONS_PER_DEGREE = Config.TURRET_REDUCTION / 360.0;


        public static final double MAX_TURN_ANGLE_DEG = MAX_TURN_POSITION / MOTOR_ROTATIONS_PER_DEGREE;

        public static final double MIN_TURN_ANGLE_DEG = MIN_TURN_POSITION / MOTOR_ROTATIONS_PER_DEGREE;


        public static final double TOLERANCE_DEG = 1.0;

        public static double degreesToMotorRotations(double degrees) {
            return degrees * MOTOR_ROTATIONS_PER_DEGREE;
        }

        public static double motorRotationsToDegrees(double rotations) {
            return rotations / MOTOR_ROTATIONS_PER_DEGREE;
        }
    }
}
