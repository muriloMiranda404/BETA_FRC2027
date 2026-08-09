package frc.robot.commands.swerveUtils.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.constants.robot.DriveConsts;
import frc.frc_java9485.utils.control.SlewRateLimiter2d;
import frc.frc_java9485.utils.logger.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class AntiTipDrive extends Command {

    private static final String LOG_KEY = "AntiTipDrive/";


    private static final LoggedTunableNumber normalSlewMps2 =
            new LoggedTunableNumber(LOG_KEY + "NormalSlewMps2", 14.0);


    private static final LoggedTunableNumber raisedCgSlewMps2 =
            new LoggedTunableNumber(LOG_KEY + "RaisedCgSlewMps2", 6.0);


    private static final LoggedTunableNumber tiltedSlewMps2 =
            new LoggedTunableNumber(LOG_KEY + "TiltedSlewMps2", 2.5);


    private static final LoggedTunableNumber tiltThresholdDeg =
            new LoggedTunableNumber(LOG_KEY + "TiltThresholdDeg", 6.0);

    private final SwerveSubsystem swerve;
    private final BooleanSupplier raisedCenterOfGravity;

    private final SlewRateLimiter2d limiter = new SlewRateLimiter2d(normalSlewMps2.get());


    public AntiTipDrive(SwerveSubsystem swerve, BooleanSupplier raisedCenterOfGravity) {
        this.swerve = swerve;
        this.raisedCenterOfGravity = raisedCenterOfGravity;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        var speeds = RobotState.getInstance().getMeasuredFieldRelativeChassisSpeeds();
        limiter.reset(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    }

    @Override
    public void execute() {
        double limit = currentSlewLimit();
        limiter.setRateLimit(limit);

        double vx = Math.pow(swerve.controller.getLeftY(), 3) * DriveConsts.MAX_SPEED;
        double vy = Math.pow(swerve.controller.getLeftX(), 3) * DriveConsts.MAX_SPEED;
        double omega = swerve.controller.getRightX() * swerve.getMaxAngularVelocity();

        Translation2d limited = limiter.calculate(vx, vy);
        swerve.driveFieldOriented(new ChassisSpeeds(limited.getX(), limited.getY(), omega));

        Logger.recordOutput(LOG_KEY + "SlewLimitMps2", limit);
        Logger.recordOutput(LOG_KEY + "RequestedMps", Math.hypot(vx, vy));
        Logger.recordOutput(LOG_KEY + "CommandedMps", limited.getNorm());
        Logger.recordOutput(LOG_KEY + "TiltDeg", measuredTiltDeg());
        Logger.recordOutput(LOG_KEY + "RaisedCg", raisedCenterOfGravity.getAsBoolean());
    }


    public double currentSlewLimit() {
        double limit = normalSlewMps2.get();

        if (raisedCenterOfGravity.getAsBoolean()) {
            limit = Math.min(limit, raisedCgSlewMps2.get());
        }
        if (measuredTiltDeg() > tiltThresholdDeg.get()) {
            limit = Math.min(limit, tiltedSlewMps2.get());
        }
        return limit;
    }


    private static double measuredTiltDeg() {
        RobotState state = RobotState.getInstance();
        double pitch = state.getPitchDegrees();
        double roll = state.getRollDegrees();


        if (!Double.isFinite(pitch) || !Double.isFinite(roll)) {
            return 0.0;
        }
        return Math.hypot(pitch, roll);
    }


    public static double slewLimitFor(boolean raisedCg, double tiltDeg) {
        double limit = normalSlewMps2.get();
        if (raisedCg) {
            limit = Math.min(limit, raisedCgSlewMps2.get());
        }
        if (Math.abs(tiltDeg) > tiltThresholdDeg.get()) {
            limit = Math.min(limit, tiltedSlewMps2.get());
        }
        return limit;
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
