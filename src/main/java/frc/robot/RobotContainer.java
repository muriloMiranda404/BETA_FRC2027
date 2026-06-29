package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.frc_java9485.autonomous.AutoChooser;
import frc.frc_java9485.joystick.driver.DriverJoystick;
import frc.frc_java9485.joystick.mechanism.MechanismJoystick;
import frc.frc_java9485.utils.RegisterNamedCommands;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.conveyor.ConveyorIOVictorSPX;
import frc.robot.subsystems.mechanism.index.IndexIOSparkMax;
import frc.robot.subsystems.mechanism.intake.IntakeIOSparkMax;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIOSparkMax;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIOSparkMax;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIOSparkMax;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {

    private RobotState robotState = new RobotState();

    private final AutoChooser autoChooser;

    private final MechanismJoystick mechanismJoystick;
    private final DriverJoystick driverJoystick;

    private final SuperStructure superStructure;

    private final SwerveSubsystem swerveSubsystem;
    private final RegisterNamedCommands namedCommands;

    public RobotContainer(){

        this.mechanismJoystick = MechanismJoystick.getInstance();
        this.driverJoystick = DriverJoystick.getInstance();

        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.namedCommands = new RegisterNamedCommands(swerveSubsystem);

        this.superStructure = new SuperStructure(
            new HoodIOSparkMax(),
            new TurretIOSparkMax(),
            new FlyWheelIOSparkMax(),
            new IntakeIOSparkMax(),
            new IndexIOSparkMax(),
            new ConveyorIOVictorSPX(),
            swerveSubsystem::getPose2d);

        this.swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
            () -> driverJoystick.getLeftY(),
            () -> driverJoystick.getLeftX(),
            () -> driverJoystick.getRightX(),
            true));

        configureAutoCommands();
        this.autoChooser = new AutoChooser("Autonomous chooser", "");
        configureBindings();
    }

    private void configureAutoCommands(){
        this.namedCommands.configureNamedCommand();
    }

    public RobotState getRobotState(){
        return robotState;
    }

    private void configureBindings(){
        
    }

    public Command getAutonomousCommand(){
        if(autoChooser.getSelectedOption() == null || autoChooser.getSelectedOption().isBlank()){
            return Commands.none();
        }
        return swerveSubsystem.getAutonomousCommand(autoChooser.getSelectedOption(), true);
    }
}
