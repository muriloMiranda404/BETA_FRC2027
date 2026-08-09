package frc.robot.commands.swerveUtils.commands;

import static frc.frc_java9485.constants.robot.DriveConsts.MAX_SPEED;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.constants.utils.FieldElementsConst;
import frc.frc_java9485.constants.utils.LoggerConstants;
import frc.frc_java9485.utils.logger.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator.ShotSolution;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AimRobotToHub extends Command {

    private static final LoggedTunableNumber kP =
            new LoggedTunableNumber("AimRobotToHub/kP", 5.0);
    private static final LoggedTunableNumber kD =
            new LoggedTunableNumber("AimRobotToHub/kD", 0.2);
    private static final LoggedTunableNumber maxAccelRadPerSec2 =
            new LoggedTunableNumber("AimRobotToHub/maxAccelRadPerSec2", 8.0);
    private static final LoggedTunableNumber toleranceDeg =
            new LoggedTunableNumber("AimRobotToHub/toleranceDeg", 2.0);
    private static final LoggedTunableNumber readyDebounceSeconds =
            new LoggedTunableNumber("AimRobotToHub/readyDebounceSeconds", 0.15);

    private final SwerveSubsystem swerve;
    private final Supplier<Translation3d> targetSupplier;
    private final boolean allowDriverTranslation;
    private final boolean finishWhenAligned;

    private final ProfiledPIDController headingController;
    private final Debouncer readyDebouncer;

    private Translation3d target;

    public AimRobotToHub(SwerveSubsystem swerve,
                         Supplier<Translation3d> targetSupplier,
                         boolean allowDriverTranslation) {
        this(swerve, targetSupplier, allowDriverTranslation, true);
    }

    public AimRobotToHub(SwerveSubsystem swerve,
                         Supplier<Translation3d> targetSupplier,
                         boolean allowDriverTranslation,
                         boolean finishWhenAligned) {
        this.swerve = swerve;
        this.targetSupplier = targetSupplier;
        this.allowDriverTranslation = allowDriverTranslation;
        this.finishWhenAligned = finishWhenAligned;

        this.headingController = new ProfiledPIDController(
                kP.get(), 0.0, kD.get(),
                new TrapezoidProfile.Constraints(swerve.getMaxAngularVelocity(), maxAccelRadPerSec2.get()));
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.headingController.setTolerance(Math.toRadians(toleranceDeg.get()));

        this.readyDebouncer = new Debouncer(readyDebounceSeconds.get(), DebounceType.kRising);

        addRequirements(swerve);
    }

    public static AimRobotToHub toHub(SwerveSubsystem swerve) {
        return new AimRobotToHub(swerve, AimRobotToHub::allianceHub, true);
    }

    public static AimRobotToHub toHubInPlace(SwerveSubsystem swerve) {
        return new AimRobotToHub(swerve, AimRobotToHub::allianceHub, false);
    }


    public static AimRobotToHub toHubAndHold(SwerveSubsystem swerve) {
        return new AimRobotToHub(swerve, AimRobotToHub::allianceHub, true, false);
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
        this.target = targetSupplier.get();

        Pose2d pose = RobotState.getInstance().getFieldToRobotPose();
        headingController.reset(pose.getRotation().getRadians());
        readyDebouncer.calculate(false);
    }

    @Override
    public void execute() {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            headingController.setP(kP.get());
            headingController.setD(kD.get());
            headingController.setConstraints(new TrapezoidProfile.Constraints(
                    swerve.getMaxAngularVelocity(), maxAccelRadPerSec2.get()));
            headingController.setTolerance(Math.toRadians(toleranceDeg.get()));
        }, kP, kD, maxAccelRadPerSec2, toleranceDeg);

        Pose2d pose = RobotState.getInstance().getFieldToRobotPose();
        double headingRad = pose.getRotation().getRadians();


        ShotSolution solution = ShotCalculator.solve(target);
        double goalHeadingRad = Math.toRadians(solution.fieldHeadingDeg());

        double omega = headingController.calculate(headingRad, goalHeadingRad);

        double vx = 0.0;
        double vy = 0.0;
        if (allowDriverTranslation) {
            vx = Math.pow(swerve.controller.getLeftY(), 3) * MAX_SPEED;
            vy = Math.pow(swerve.controller.getLeftX(), 3) * MAX_SPEED;
        }
        swerve.driveFieldOriented(new ChassisSpeeds(vx, vy, omega));

        Logger.recordOutput(LoggerConstants.AIM_ROBOT_TO_HUB + "Target", target);
        Logger.recordOutput(LoggerConstants.AIM_ROBOT_TO_HUB + "GoalHeadingDeg", Math.toDegrees(goalHeadingRad));
        Logger.recordOutput(LoggerConstants.AIM_ROBOT_TO_HUB + "MeasuredHeadingDeg", Math.toDegrees(headingRad));
        Logger.recordOutput(LoggerConstants.AIM_ROBOT_TO_HUB + "ErrorDeg",
                Math.toDegrees(MathUtil.angleModulus(goalHeadingRad - headingRad)));
        Logger.recordOutput(LoggerConstants.AIM_ROBOT_TO_HUB + "OmegaRadPerSec", omega);
        Logger.recordOutput(LoggerConstants.AIM_ROBOT_TO_HUB + "AtGoal", headingController.atGoal());
    }

    @Override
    public boolean isFinished() {
        return finishWhenAligned && readyDebouncer.calculate(headingController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
        Logger.recordOutput(LoggerConstants.AIM_ROBOT_TO_HUB + "AtGoal", false);
    }
}
