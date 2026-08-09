package frc.robot.commands.swerveUtils.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.constants.mechanisms.shooter.ShotVerifierConsts;
import frc.frc_java9485.constants.utils.FieldElementsConst;
import frc.frc_java9485.constants.utils.FieldElementsConst.FieldMeansureds;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class DriveIntoShotRange extends Command {

    private static final String LOG_KEY = "DriveIntoShotRange/";


    public static final double RANGE_MARGIN_M = 0.35;


    private static final double FIELD_EDGE_MARGIN_M = 0.6;

    private final SwerveSubsystem swerve;
    private final Supplier<Translation3d> targetSupplier;

    private Translation3d target;
    private Pose2d goalPose;

    public DriveIntoShotRange(SwerveSubsystem swerve, Supplier<Translation3d> targetSupplier) {
        this.swerve = swerve;
        this.targetSupplier = targetSupplier;
        addRequirements(swerve);
    }

    public static DriveIntoShotRange toHub(SwerveSubsystem swerve) {
        return new DriveIntoShotRange(swerve, DriveIntoShotRange::allianceHub);
    }

    private static Translation3d allianceHub() {
        return DriverStation.getAlliance()
                .map(a -> a == Alliance.Red
                        ? FieldElementsConst.HubMeansured.HUB_RED
                        : FieldElementsConst.HubMeansured.HUB_BLUE)
                .orElse(FieldElementsConst.HubMeansured.HUB_BLUE);
    }




    public static Pose2d nearestShootingPose(
            Translation2d robot, Translation2d hub, double minDistance, double maxDistance, double margin) {

        double innerBound = minDistance + margin;
        double outerBound = maxDistance - margin;

        if (innerBound > outerBound) {
            innerBound = (minDistance + maxDistance) / 2.0;
            outerBound = innerBound;
        }

        Translation2d hubToRobot = robot.minus(hub);
        double distance = hubToRobot.getNorm();


        Translation2d direction = distance < 1e-6
                ? new Translation2d(1.0, 0.0)
                : hubToRobot.div(distance);

        double targetDistance = MathUtil.clamp(distance, innerBound, outerBound);
        Translation2d position = hub.plus(direction.times(targetDistance));

        return new Pose2d(position, headingToward(position, hub));
    }


    public static boolean isInRange(Translation2d robot, Translation2d hub) {
        double distance = robot.getDistance(hub);
        return distance >= ShotVerifierConsts.MIN_SHOT_DISTANCE_M
                && distance <= ShotVerifierConsts.MAX_SHOT_DISTANCE_M;
    }

    private static Rotation2d headingToward(Translation2d from, Translation2d to) {
        Translation2d delta = to.minus(from);
        return delta.getNorm() < 1e-6 ? Rotation2d.kZero : delta.getAngle();
    }


    private static Pose2d clampToField(Pose2d pose) {
        double x = MathUtil.clamp(pose.getX(), FIELD_EDGE_MARGIN_M,
                FieldMeansureds.FIELD_LENGTH_METERS - FIELD_EDGE_MARGIN_M);
        double y = MathUtil.clamp(pose.getY(), FIELD_EDGE_MARGIN_M,
                FieldMeansureds.FIELD_WIDTH_METERS - FIELD_EDGE_MARGIN_M);
        return new Pose2d(x, y, pose.getRotation());
    }



    @Override
    public void initialize() {
        target = targetSupplier.get();
        swerve.resetDriveToPoseControllers();
        goalPose = computeGoal();
    }

    @Override
    public void execute() {

        goalPose = computeGoal();
        swerve.driveToPose(goalPose);

        Logger.recordOutput(LOG_KEY + "GoalPose", goalPose);
        Logger.recordOutput(LOG_KEY + "DistanceToHubM", distanceToHub());
        Logger.recordOutput(LOG_KEY + "InRange", isInRange(robotTranslation(), hubTranslation()));
    }

    private Pose2d computeGoal() {
        return clampToField(nearestShootingPose(
                robotTranslation(),
                hubTranslation(),
                ShotVerifierConsts.MIN_SHOT_DISTANCE_M,
                ShotVerifierConsts.MAX_SHOT_DISTANCE_M,
                RANGE_MARGIN_M));
    }

    private Translation2d robotTranslation() {
        return swerve.getPose2d().getTranslation();
    }

    private Translation2d hubTranslation() {
        return new Translation2d(target.getX(), target.getY());
    }

    private double distanceToHub() {
        return robotTranslation().getDistance(hubTranslation());
    }

    @Override
    public boolean isFinished() {
        return swerve.atTargetPose();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true);
    }
}
