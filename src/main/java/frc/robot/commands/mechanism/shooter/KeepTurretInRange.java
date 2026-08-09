package frc.robot.commands.mechanism.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.constants.robot.DriveConsts;
import frc.frc_java9485.utils.logger.LoggedTunableNumber;
import frc.robot.commands.mechanism.shooter.TurretChassisAllocator.Allocation;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class KeepTurretInRange extends Command {

    private static final String LOG_KEY = "KeepTurretInRange/";

    private static final LoggedTunableNumber kP = new LoggedTunableNumber(LOG_KEY + "kP", 4.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber(LOG_KEY + "kD", 0.1);

    private final SwerveSubsystem swerve;
    private final Supplier<Translation3d> targetSupplier;
    private final boolean allowDriverTranslation;

    private final PIDController headingController = new PIDController(kP.get(), 0.0, kD.get());


    private boolean chassisEngaged = false;

    private Translation3d target;

    public KeepTurretInRange(
            SwerveSubsystem swerve, Supplier<Translation3d> targetSupplier, boolean allowDriverTranslation) {
        this.swerve = swerve;
        this.targetSupplier = targetSupplier;
        this.allowDriverTranslation = allowDriverTranslation;

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        target = targetSupplier.get();
        chassisEngaged = false;
        headingController.reset();
    }

    @Override
    public void execute() {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            headingController.setP(kP.get());
            headingController.setD(kD.get());
        }, kP, kD);

        double desiredFieldHeadingDeg = ShotCalculator.solve(target).fieldHeadingDeg();
        double chassisHeadingDeg = swerve.getPose2d().getRotation().getDegrees();

        Allocation allocation =
                TurretChassisAllocator.allocate(desiredFieldHeadingDeg, chassisHeadingDeg, chassisEngaged);
        chassisEngaged = allocation.chassisEngaged();


        double omega = 0.0;
        if (allocation.chassisEngaged()) {
            omega = headingController.calculate(
                    Math.toRadians(chassisHeadingDeg), Math.toRadians(allocation.chassisGoalHeadingDeg()));
            omega = MathUtil.clamp(omega, -swerve.getMaxAngularVelocity(), swerve.getMaxAngularVelocity());
        }

        double vx = 0.0;
        double vy = 0.0;
        if (allowDriverTranslation) {
            vx = Math.pow(swerve.controller.getLeftY(), 3) * DriveConsts.MAX_SPEED;
            vy = Math.pow(swerve.controller.getLeftX(), 3) * DriveConsts.MAX_SPEED;
        }
        swerve.driveFieldOriented(new ChassisSpeeds(vx, vy, omega));

        Logger.recordOutput(LOG_KEY + "DesiredFieldHeadingDeg", desiredFieldHeadingDeg);
        Logger.recordOutput(LOG_KEY + "TurretCommandDeg", allocation.turretRelativeDeg());
        Logger.recordOutput(LOG_KEY + "ChassisEngaged", allocation.chassisEngaged());
        Logger.recordOutput(LOG_KEY + "TurretSaturated", allocation.turretSaturated());
        Logger.recordOutput(LOG_KEY + "RemainingTravelDeg",
                TurretChassisAllocator.remainingTravelDeg(allocation.turretRelativeDeg()));
        Logger.recordOutput(LOG_KEY + "OmegaRadPerSec", omega);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true);
        chassisEngaged = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
