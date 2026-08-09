package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import static frc.frc_java9485.constants.robot.RobotConsts.isSimulation;

import frc.robot.commands.mechanism.climber.GatedClimb;
import frc.robot.commands.diagnostics.PitDiagnostics;
import frc.robot.commands.mechanism.shooter.KeepTurretInRange;
import frc.robot.commands.mechanism.shooter.ShiftAwareShooting;
import frc.robot.commands.mechanism.shooter.ShootOnTheMove;
import frc.robot.commands.mechanism.shooter.ShotCorrectionPolicy;
import frc.robot.commands.swerveUtils.commands.AimRobotToHub;
import frc.robot.commands.swerveUtils.commands.AntiTipDrive;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.led.LedSubsystem;

import frc.frc_java9485.autonomous.AutoChooser;
import frc.frc_java9485.autonomous.AutoManager;
import frc.frc_java9485.joystick.driver.DriverJoystick;
import frc.frc_java9485.joystick.mechanism.MechanismJoystick;
import frc.frc_java9485.autonomous.RegisterNamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.mechanism.MechanismSim;
import frc.robot.subsystems.mechanism.MechanismVisualizer;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.conveyor.ConveyorIOSim;
import frc.robot.subsystems.mechanism.conveyor.ConveyorIOTalonFX;
import frc.robot.subsystems.mechanism.index.IndexIOSim;
import frc.robot.subsystems.mechanism.index.IndexIOTalonFX;
import frc.robot.subsystems.mechanism.intake.IntakeIOSim;
import frc.robot.subsystems.mechanism.intake.IntakeIOTalonFX;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIOSim;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIOTalonFX;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIOSim;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIOSim;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

    private RobotState robotState = RobotState.getInstance();

    private final AutoChooser autoChooser;
    private final AutoManager autoManager;

    private final MechanismJoystick mechanismJoystick;
    private final DriverJoystick driverJoystick;

    private final SuperStructure superStructure;
    private MechanismSim mechanismSim;
    private final LedSubsystem ledSubsystem;
    private final ClimberSubsystem climberSubsystem;

    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final RegisterNamedCommands namedCommands;

    public RobotContainer(){

        this.mechanismJoystick = MechanismJoystick.getInstance();
        this.driverJoystick = DriverJoystick.getInstance();

        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.visionSubsystem = new VisionSubsystem(swerveSubsystem);

        this.superStructure = isSimulation()
            ? new SuperStructure(
                new HoodIOSim(),
                new TurretIOSim(),
                new FlyWheelIOSim(),
                new IntakeIOSim(),
                new IndexIOSim(),
                new ConveyorIOSim())
            : new SuperStructure(
                new HoodIOTalonFX(),
                new TurretIOTalonFX(),
                new FlyWheelIOTalonFX(),
                new IntakeIOTalonFX(),
                new IndexIOTalonFX(),
                new ConveyorIOTalonFX());

        if (isSimulation()) {
            this.mechanismSim = new MechanismSim(superStructure, swerveSubsystem);
        }

        this.climberSubsystem = new ClimberSubsystem(
            isSimulation() ? new ClimberIOSim() : new ClimberIOTalonFX());

        this.namedCommands = new RegisterNamedCommands(swerveSubsystem, superStructure, climberSubsystem);


        new MechanismVisualizer(
            superStructure::getTurretAngleDeg,
            superStructure::getHoodPosition,
            superStructure::getIntakePivotPosition,
            climberSubsystem::getPosition,
            () -> new Pose3d(swerveSubsystem.getPose2d()));

        this.ledSubsystem = LedSubsystem.getInstance();
        configureLeds();

        this.swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
            () -> driverJoystick.getLeftY(),
            () -> driverJoystick.getLeftX(),
            () -> driverJoystick.getRightX(),
            true));

        configureAutoCommands();
        this.autoChooser = new AutoChooser("Autonomous chooser", AutoChooser.NONE_OPTION);

        this.autoManager = new AutoManager(autoChooser, swerveSubsystem::getPose2d);
        configureBindings();
    }

    private void configureAutoCommands(){
        this.namedCommands.configureNamedCommand();
    }


    private void configureLeds(){
        ledSubsystem.setDefaultCommand(ledSubsystem.run(() -> {
            Color color;
            if (superStructure.isReadyToShoot()) {
                color = Color.kGreen;
            } else {
                color = switch (superStructure.getCurrentState()) {
                    case COLLECTING -> Color.kYellow;

                    case PREPARING -> Color.kOrange;
                    case SHOOTING, PASSING -> Color.kBlue;
                    case EJECTING_BY_INTAKE -> Color.kPurple;
                    case OFF -> Color.kRed;
                };
            }
            ledSubsystem.setSolidColor(color);
        }));
    }

    public RobotState getRobotState(){
        return robotState;
    }


    public void updateSimulation(){
        if (mechanismSim != null) {
            mechanismSim.update();
        }
    }

    private void configureBindings(){

        mechanismJoystick.rightTrigger().whileTrue(superStructure.shoot());
        mechanismJoystick.leftBumper().whileTrue(superStructure.collect());
        mechanismJoystick.b().whileTrue(superStructure.eject());
        mechanismJoystick.y().whileTrue(superStructure.pass());


        mechanismJoystick.a().whileTrue(
            AimRobotToHub.toHubAndHold(swerveSubsystem).alongWith(superStructure.shoot()));


        mechanismJoystick.getUpPOV().whileTrue(GatedClimb.extend(climberSubsystem, superStructure));
        mechanismJoystick.getDownPOV().whileTrue(GatedClimb.retract(climberSubsystem, superStructure));


        mechanismJoystick.x().whileTrue(ShiftAwareShooting.collectingWhenClosed(superStructure));


        driverJoystick.a().onTrue(Commands.runOnce(() ->
            swerveSubsystem.resetOdometry(
                new Pose2d(swerveSubsystem.getPose2d().getTranslation(), new Rotation2d())),
            swerveSubsystem));


        driverJoystick.y().whileTrue(AimRobotToHub.toHubAndHold(swerveSubsystem));


        driverJoystick.x().whileTrue(ShootOnTheMove.toHub(swerveSubsystem, superStructure));


        driverJoystick.rightBumper().and(driverJoystick.y()).whileTrue(
            ShotCorrectionPolicy.shootWithCorrection(swerveSubsystem, superStructure));


        driverJoystick.getLeftBack().whileTrue(
            new KeepTurretInRange(swerveSubsystem, superStructure::getCurrentTarget, true));


        RobotModeTriggers.teleop().and(driverJoystick.b()).whileTrue(
            swerveSubsystem.pivotAroundTurretCommand(
                () -> driverJoystick.getLeftY(),
                () -> driverJoystick.getLeftX(),
                () -> driverJoystick.getRightX()));


        RobotModeTriggers.test().and(driverJoystick.b())
            .whileTrue(swerveSubsystem.sysIdDriveCommand());
        RobotModeTriggers.test().and(driverJoystick.getLeftBack())
            .whileTrue(swerveSubsystem.sysIdAngleCommand());


        RobotModeTriggers.test().and(mechanismJoystick.a())
            .onTrue(PitDiagnostics.runAll(superStructure, climberSubsystem));
    }


    public Command antiTipDriveCommand(){
        return new AntiTipDrive(swerveSubsystem, () ->
            superStructure.getCurrentState() == SuperStructure.SystemState.COLLECTING
                || climberSubsystem.getPosition() > 5.0);
    }

    public Command getAutonomousCommand(){

        return autoManager.getAutonomousCommand();
    }
}
