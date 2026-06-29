package frc.frc_java9485.utils;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.swerveUtils.DriveToSupportPoint;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RegisterNamedCommands {

    private final SwerveSubsystem swerveSubsystem;

    public RegisterNamedCommands(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
    }

    public void configureNamedCommand(){
        configureDriveCommands(swerveSubsystem);
    }

    private void configureDriveCommands(SwerveSubsystem swerveSubsystem){
        NamedCommands.registerCommand("Drive to support point", new DriveToSupportPoint(swerveSubsystem));
    }
}
