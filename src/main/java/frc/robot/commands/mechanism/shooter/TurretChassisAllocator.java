package frc.robot.commands.mechanism.shooter;

import edu.wpi.first.math.MathUtil;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;


public class TurretChassisAllocator {


    public static final double ENGAGE_MARGIN_DEG = 15.0;


    public static final double RELEASE_MARGIN_DEG = 40.0;

    private TurretChassisAllocator() {}


    public record Allocation(
            double turretRelativeDeg,
            double chassisGoalHeadingDeg,
            boolean chassisEngaged,
            boolean turretSaturated) {}


    public static Allocation allocate(
            double desiredFieldHeadingDeg, double chassisHeadingDeg, boolean chassisWasEngaged) {

        double desiredTurretRel =
                MathUtil.inputModulus(desiredFieldHeadingDeg - chassisHeadingDeg, -180.0, 180.0);

        double min = TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG;
        double max = TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG;

        boolean engaged = chassisWasEngaged
                ? !isComfortablyInside(desiredTurretRel, min, max, RELEASE_MARGIN_DEG)
                : !isComfortablyInside(desiredTurretRel, min, max, ENGAGE_MARGIN_DEG);


        double chassisGoal = engaged ? desiredFieldHeadingDeg : chassisHeadingDeg;

        double turretCommand = MathUtil.clamp(desiredTurretRel, min, max);
        boolean saturated = turretCommand != desiredTurretRel;

        return new Allocation(turretCommand, chassisGoal, engaged, saturated);
    }


    private static boolean isComfortablyInside(double angleDeg, double min, double max, double margin) {
        return angleDeg >= min + margin && angleDeg <= max - margin;
    }


    public static double remainingTravelDeg(double turretRelativeDeg) {
        double min = TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG;
        double max = TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG;
        return Math.min(turretRelativeDeg - min, max - turretRelativeDeg);
    }
}
