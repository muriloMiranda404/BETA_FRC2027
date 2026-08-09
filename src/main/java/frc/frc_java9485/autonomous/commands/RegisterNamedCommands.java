package frc.frc_java9485.autonomous;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.swerveUtils.commands.DriveToSupportPoint;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class RegisterNamedCommands {

    private final SwerveSubsystem swerveSubsystem;
    private final SuperStructure superStructure;
    private final ClimberSubsystem climber;

    public RegisterNamedCommands(SwerveSubsystem swerveSubsystem, SuperStructure superStructure,
                                 ClimberSubsystem climber){
        this.swerveSubsystem = swerveSubsystem;
        this.superStructure = superStructure;
        this.climber = climber;
    }

    public void configureNamedCommand(){
        configureDriveCommands();
        configureMechanismCommands();
        configureLegacyAliases();
    }

    private void configureDriveCommands(){
        NamedCommands.registerCommand("Drive to support point", new DriveToSupportPoint(swerveSubsystem));
        NamedCommands.registerCommand("AimAtHub", AutoCommands.aimAtHub(swerveSubsystem));
    }

    private void configureMechanismCommands(){
        NamedCommands.registerCommand("Shoot", AutoCommands.shootWhenReady(superStructure));
        NamedCommands.registerCommand("AimAndShoot", AutoCommands.aimAndShoot(swerveSubsystem, superStructure));
        NamedCommands.registerCommand("WaitForShotReady", AutoCommands.waitForShotReady(superStructure));
        NamedCommands.registerCommand("Collect", AutoCommands.collect(superStructure));
        NamedCommands.registerCommand("Pass", AutoCommands.pass(superStructure));
        NamedCommands.registerCommand("Eject", AutoCommands.eject(superStructure));
        NamedCommands.registerCommand("Climb", AutoCommands.climb(climber));
    }


    private void configureLegacyAliases(){
        NamedCommands.registerCommand("coleta", AutoCommands.collect(superStructure));
        NamedCommands.registerCommand("down intake", AutoCommands.collect(superStructure));
        NamedCommands.registerCommand("open conveyor", AutoCommands.collect(superStructure));
        NamedCommands.registerCommand("shootar", AutoCommands.shootWhenReady(superStructure));
        NamedCommands.registerCommand("shot fuel center",
            AutoCommands.aimAndShoot(swerveSubsystem, superStructure));
        NamedCommands.registerCommand("turn to center", AutoCommands.aimAtHub(swerveSubsystem));
        NamedCommands.registerCommand("Climbar", AutoCommands.climb(climber));
    }
}
