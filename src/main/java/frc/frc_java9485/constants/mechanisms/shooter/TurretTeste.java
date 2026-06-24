package frc.frc_java9485.constants.mechanisms.shooter;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class TurretTeste {
  public static final class Motors{
    public static final int TURN_TURRET = 13;
  }

  public static final class Configs{
    public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
    new Transform3d(new Translation3d(Inches.zero(), Inches.of(7), Inches.of(17.5)), Rotation3d.kZero);

    public static final int TURN_TURRET_CURRENT_LIMIT = 20;

    public static final double TURRET_REDUCTION = 66.8;
  }

  public static final class Setpoint{
    public static final double MAX_TURN_POSITION = 17.78;
    public static final double MIN_TURN_POSITION = -19.69;
  }

  public static final class PID{
    public static final ControlConstants SHOOTER_CONTROL_CONSTANTS = new ControlConstants()
    .withPID(2.3, 1, 0.1)
    .withTolerance(21);

    public static final TunableControlConstants SHOOTER_CONSTANTS = new TunableControlConstants("shooter controller", SHOOTER_CONTROL_CONSTANTS);

    public static final ControlConstants TURRET_MANUAL_CONSTANTS = new ControlConstants()
    .withPID(0.075, 0.05, 0.0)
    .withProfile(250, 170);

    public static final ControlConstants AUTOMATIC_TURRET = new ControlConstants()
    .withPID(0.2, 0.0, 0.0)
    .withTolerance(0.1)
    .withProfile(3, 8)
    .withFeedforward(0.08, 0)
    .withPhysical(2.0, 0);

    public static final TunableControlConstants AUTOMATIC_TURRET_CONTROL = new TunableControlConstants("automatic turret", AUTOMATIC_TURRET);

    public static final TunableControlConstants TURRET_TUNABLE = new TunableControlConstants("shooter tunableConstants", TURRET_MANUAL_CONSTANTS);
  }
}
