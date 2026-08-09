package frc.robot.commands.mechanism.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.constants.robot.DriveConsts;
import frc.frc_java9485.constants.utils.FieldElementsConst;
import frc.frc_java9485.utils.control.SlewRateLimiter2d;
import frc.frc_java9485.utils.logger.LoggedTunableNumber;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator.ShotSolution;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class ShootOnTheMove extends Command {

    private static final String LOG_KEY = "ShootOnTheMove/";

    private static final LoggedTunableNumber kP =
            new LoggedTunableNumber(LOG_KEY + "kP", 4.5);
    private static final LoggedTunableNumber kD =
            new LoggedTunableNumber(LOG_KEY + "kD", 0.15);


    private static final LoggedTunableNumber kFeedforward =
            new LoggedTunableNumber(LOG_KEY + "kFeedforward", 1.0);


    private static final LoggedTunableNumber translationSlewMps2 =
            new LoggedTunableNumber(LOG_KEY + "translationSlewMps2", 6.0);


    private static final int GOAL_RATE_FILTER_TAPS = 5;

    private static final double LOOP_PERIOD_SEC = 0.02;

    private final SwerveSubsystem swerve;
    private final Supplier<Translation3d> targetSupplier;

    private final PIDController headingController = new PIDController(kP.get(), 0.0, kD.get());
    private final LinearFilter goalRateFilter = LinearFilter.movingAverage(GOAL_RATE_FILTER_TAPS);
    private final SlewRateLimiter2d translationLimiter =
            new SlewRateLimiter2d(translationSlewMps2.get());

    private Translation3d target;
    private double lastGoalHeadingRad = Double.NaN;

    public ShootOnTheMove(SwerveSubsystem swerve, Supplier<Translation3d> targetSupplier) {
        this.swerve = swerve;
        this.targetSupplier = targetSupplier;

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }


    public static Command toHub(SwerveSubsystem swerve, SuperStructure superStructure) {
        return new ShootOnTheMove(swerve, ShootOnTheMove::allianceHub)
                .alongWith(superStructure.shoot())
                .withName("ShootOnTheMove");
    }

    private static Translation3d allianceHub() {
        return DriverStation.getAlliance()
                .map(a -> a == Alliance.Red
                        ? FieldElementsConst.HubMeansured.HUB_RED
                        : FieldElementsConst.HubMeansured.HUB_BLUE)
                .orElse(FieldElementsConst.HubMeansured.HUB_BLUE);
    }

    @Override
    public void initialize() {
        target = targetSupplier.get();
        headingController.reset();
        goalRateFilter.reset();
        lastGoalHeadingRad = Double.NaN;
        translationLimiter.reset(new Translation2d());
    }

    @Override
    public void execute() {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            headingController.setP(kP.get());
            headingController.setD(kD.get());
            translationLimiter.setRateLimit(translationSlewMps2.get());
        }, kP, kD, translationSlewMps2);

        ShotSolution solution = ShotCalculator.solve(target);
        double goalHeadingRad = Math.toRadians(solution.fieldHeadingDeg());
        double measuredHeadingRad = swerve.getPose2d().getRotation().getRadians();

        double goalRateRadPerSec = measureGoalRate(goalHeadingRad);
        double feedback = headingController.calculate(measuredHeadingRad, goalHeadingRad);
        double omega = feedback + goalRateRadPerSec * kFeedforward.get();

        omega = MathUtil.clamp(omega, -swerve.getMaxAngularVelocity(), swerve.getMaxAngularVelocity());

        Translation2d translation = limitedDriverTranslation();
        swerve.driveFieldOriented(new ChassisSpeeds(translation.getX(), translation.getY(), omega));

        Logger.recordOutput(LOG_KEY + "GoalHeadingDeg", Math.toDegrees(goalHeadingRad));
        Logger.recordOutput(LOG_KEY + "MeasuredHeadingDeg", Math.toDegrees(measuredHeadingRad));
        Logger.recordOutput(LOG_KEY + "HeadingErrorDeg",
                Math.toDegrees(MathUtil.angleModulus(goalHeadingRad - measuredHeadingRad)));
        Logger.recordOutput(LOG_KEY + "GoalRateDegPerSec", Math.toDegrees(goalRateRadPerSec));
        Logger.recordOutput(LOG_KEY + "FeedbackOmega", feedback);
        Logger.recordOutput(LOG_KEY + "OmegaRadPerSec", omega);
        Logger.recordOutput(LOG_KEY + "CompensatedDistanceM", solution.compensatedDistanceM());
    }


    private double measureGoalRate(double goalHeadingRad) {
        if (Double.isNaN(lastGoalHeadingRad)) {
            lastGoalHeadingRad = goalHeadingRad;
            return 0.0;
        }

        double delta = MathUtil.angleModulus(goalHeadingRad - lastGoalHeadingRad);
        lastGoalHeadingRad = goalHeadingRad;

        return goalRateFilter.calculate(delta / LOOP_PERIOD_SEC);
    }


    private Translation2d limitedDriverTranslation() {
        double vx = Math.pow(swerve.controller.getLeftY(), 3) * DriveConsts.MAX_SPEED;
        double vy = Math.pow(swerve.controller.getLeftX(), 3) * DriveConsts.MAX_SPEED;
        return translationLimiter.calculate(vx, vy);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true);
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}
