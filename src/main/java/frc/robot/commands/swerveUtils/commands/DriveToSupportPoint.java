package frc.robot.commands.swerveUtils.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveToSupportPoint extends Command{

    private final SwerveSubsystem swerveSubsystem;

    public DriveToSupportPoint(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.resetDriveToPoseControllers();
    }

    @Override
    public void execute() {
        swerveSubsystem.driveToSupportPoint();
    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.atTargetPose();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
