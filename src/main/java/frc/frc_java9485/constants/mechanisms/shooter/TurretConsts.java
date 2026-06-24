package frc.frc_java9485.constants.mechanisms.shooter;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class TurretConsts {

    public static final class Motors{
        public static final int TURRET_MOTOR = 13;
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
    }
}
