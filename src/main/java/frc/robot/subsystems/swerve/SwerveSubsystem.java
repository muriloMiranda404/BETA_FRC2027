package frc.robot.subsystems.swerve;

import static frc.frc_java9485.constants.robot.ComponentsConsts.*;
import static frc.frc_java9485.constants.robot.DriveConsts.*;
import static frc.frc_java9485.constants.robot.RobotConsts.CURRENT_ROBOT_MODE;
import static frc.frc_java9485.constants.robot.RobotConsts.isSimulation;
import static frc.frc_java9485.constants.utils.FieldElementsConst.FieldMeansureds.*;

import java.io.File;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.frc_java9485.constants.robot.RobotConsts.RobotModes;
import frc.frc_java9485.constants.utils.FieldElementsConst;
import frc.frc_java9485.joystick.driver.DriverJoystick;
import frc.frc_java9485.loggers.CustomDoubleLogger;
import frc.frc_java9485.utils.calc.MathUtils;
import frc.frc_java9485.utils.Rebuilt.AllianceFlip;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.IO.SwerveIO;
import frc.robot.subsystems.swerve.IO.SwerveInputsAutoLogged;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.GyroSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase implements SwerveIO {
  public static final Lock odometryLock = new ReentrantLock();
  public final DriverJoystick controller = DriverJoystick.getInstance();

  private double targetHeadingDegrees = Double.NaN;
  private double lastDesiredJoystickAngle;

  private final ProfiledPIDController moveToPoseXAxisPid;
  private final ProfiledPIDController moveToPoseYAxisPid;

  private final SwerveDrive swerveDrive;

  private final SwerveInputsAutoLogged swerveInputs;

  private final Pigeon2 pigeon;

  private final CANcoder[] encoders;

  private final CustomDoubleLogger targetVxDriveToPoseLogger = new CustomDoubleLogger("/Swerve/targetVxSpeed");
  private final CustomDoubleLogger targetVyDriveToPoseLogger = new CustomDoubleLogger("/Swerve/targetVySpeed");

  private String state = "NUll";
  private Pose2d targetPose = null;

  private SwerveDriveSimulation driveSimulator;



  private SwerveModuleState states[];

  private static SwerveSubsystem mInstance;

  private final boolean isSimulation;


  private static final Translation2d TURRET_PIVOT =
      TurretConsts.Config.ROBOT_TO_TURRET_TRANSFORM.getTranslation().toTranslation2d();

  public static SwerveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    }
    return mInstance;
  }

  private SwerveSubsystem(File directory) {
    try {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

      swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED);
      swerveDrive.setHeadingCorrection(true);
      swerveDrive.setCosineCompensator(true);
      swerveDrive.setAngularVelocityCompensation(true,
                                                 false,
                                                 0.01);

      swerveDrive.setMotorIdleMode(false);

      if (CURRENT_ROBOT_MODE == RobotModes.SIM) {
        isSimulation = true;
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);

        driveSimulator = swerveDrive.getMapleSimDrive().get();
        driveSimulator.setEnabled(true);

        driveSimulator.config.gyroSimulationFactory = COTS.ofPigeon2();

      } else {
        isSimulation = false;
      }

      this.moveToPoseXAxisPid = new ProfiledPIDController(0.01, 0, 0,
        new TrapezoidProfile.Constraints(swerveDrive.getMaximumChassisVelocity(), 2));
      this.moveToPoseYAxisPid = new ProfiledPIDController(0.01, 0, 0,
        new TrapezoidProfile.Constraints(swerveDrive.getMaximumChassisVelocity(), 2));

      this.moveToPoseXAxisPid.setTolerance(0.03);
      this.moveToPoseYAxisPid.setTolerance(0.03);

      encoders =
          new CANcoder[] {
            new CANcoder(CANCODER_MODULE1_ID),
            new CANcoder(CANCODER_MODULE2_ID),
            new CANcoder(CANCODER_MODULE3_ID),
            new CANcoder(CANCODER_MODULE4_ID)
          };

      pigeon = new Pigeon2(PIGEON2);
      pigeon.reset();

      swerveInputs = new SwerveInputsAutoLogged();

      setupPathPlanner();

      this.lastDesiredJoystickAngle = AllianceFlip.shouldFlip() ? 0 : 180;

    } catch (Exception e) {
      throw new RuntimeException("Erro criando Swerve!!!!\n", e);
    }
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    updateInputs(swerveInputs);

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getPose2d().getRotation());
    RobotState.getInstance()
        .addOdometryMeasurement(Timer.getFPGATimestamp(), getPose2d(), fieldRelativeSpeeds);
    RobotState.getInstance().setRobotAttitude(getPitch(), getRoll());

    Logger.recordOutput("Target Joystick Angle", targetHeadingDegrees);
    Logger.recordOutput("last Desired Joystick Angle", lastDesiredJoystickAngle);
    Logger.processInputs("Swerve", swerveInputs);
  }

  @Override
  public Pose2d getPose2d() {
    return swerveDrive.getPose();
  }

  @Override
  public Rotation2d getHeading2d() {
    return isSimulation ?
      driveSimulator.getGyroSimulation().getGyroReading() :
      Rotation2d.fromDegrees(MathUtils.scope0To360(pigeon.getYaw().getValueAsDouble()));
  }

  @Override
  public Rotation3d getHeading3d() {
    return isSimulation ?
      new Rotation3d(driveSimulator.getGyroSimulation().getGyroReading()) :
      pigeon.getRotation3d();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    if (this.swerveDrive != null) {
      this.swerveDrive.resetOdometry(pose);
    }
    if (isSimulation()) {
      this.driveSimulator.setSimulationWorldPose(pose);
    }
  }

  @Override
  public SwerveDriveSimulation getSimulation() {
    return this.driveSimulator;
  }

  @Override
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return isSimulation() ?
           driveSimulator.getDriveTrainSimulatedChassisSpeedsRobotRelative() :
           swerveDrive.getRobotVelocity();
  }

  @Override
  public void driveFieldOriented(ChassisSpeeds speed) {
    this.swerveDrive.driveFieldOriented(speed);
  }

  @Override
  public Pigeon2 getPigeon() {
    return this.pigeon;
  }

  @Override
  public GyroSimulation getGyroSimulation() {
    return this.driveSimulator.getGyroSimulation();
  }

  @Override
  public void drive(Translation2d translation2d, double rotation, boolean fieldOriented) {
    this.swerveDrive.drive(translation2d, rotation, fieldOriented, false);
  }

  @Override
  public double getMaxAngularVelocity() {
    return this.swerveDrive.getMaximumChassisAngularVelocity();
  }

  protected ChassisSpeeds inputsToChassisSpeeds(double xInput, double yInput) {
    return new ChassisSpeeds(xInput * swerveDrive.getMaximumChassisVelocity(), yInput * swerveDrive.getMaximumChassisVelocity(), 0);
  }

  protected ChassisSpeeds inputsToChassisSpeeds(double xInput, double yInput, double AngularRate) {
    return new ChassisSpeeds(xInput * swerveDrive.getMaximumChassisVelocity(),
                             yInput * swerveDrive.getMaximumChassisVelocity(),
                             AngularRate * swerveDrive.getMaximumChassisAngularVelocity());
  }


  public void driveRotating(boolean rotateRight) {
      ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getLeftY(),
          controller.getLeftX(), rotateRight ? -1.5 : 1.5);
      this.state = "DRIVE_ALIGN_ANGLE_ROTATING_RIGHT?:" + Boolean.toString(rotateRight);
      this.driveFieldOriented(desiredSpeeds);
  }

  @Override
  public void lock() {
    this.swerveDrive.lockPose();
  }

  @Override
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    this.swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    this.swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  @Override
  public Command getAutonomousCommand(String path, boolean altern) {
    if (altern) {
      return AutoBuilder.buildAuto(path);
    }
    return new PathPlannerAuto(path);
  }

  @Override
  public Command driveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier omega, boolean fieldOriented) {
    return run(() -> {
      double vx  = Math.pow(X.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity();
      double vy  = Math.pow(Y.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity();
      double rot = omega.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity();

      ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rot);

      if (fieldOriented) {
        swerveDrive.driveFieldOriented(speeds);
      } else {
        swerveDrive.drive(speeds);
      }
    });
  }


  public void driveToPose(Pose2d targetPose){
    this.driveToPose(targetPose, Double.POSITIVE_INFINITY);
  }

  public void driveToPose(Pose2d targetPose, double maxSpeed) {
    Pose2d currentPose = getPose2d();
    double targetXVelocity = this.moveToPoseXAxisPid.calculate(currentPose.getX(),
        (targetPose.getX()));
    double targetYVelocity = this.moveToPoseYAxisPid.calculate(currentPose.getY(),
        (targetPose.getY()));
    double targetTranslationVelocity = new Translation2d(targetXVelocity, targetYVelocity).getNorm();
    if (targetTranslationVelocity > maxSpeed) {
      double conversionFactor = maxSpeed / targetTranslationVelocity;
      targetXVelocity = targetXVelocity * conversionFactor;
      targetYVelocity = targetYVelocity * conversionFactor;
    }
    this.targetVxDriveToPoseLogger.append(targetXVelocity);
    this.targetVyDriveToPoseLogger.append(targetYVelocity);

    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(targetXVelocity, targetYVelocity, 0);
    this.driveFieldOrientedLockedAngle(desiredSpeeds, targetPose.getRotation());
    this.targetPose = targetPose;
  }

  @Override
  public boolean atTargetPose() {
    if (targetPose == null) return false;

    double headingError = MathUtil.angleModulus(
        targetPose.getRotation().getRadians() - getPose2d().getRotation().getRadians());

    return moveToPoseXAxisPid.atGoal() && moveToPoseYAxisPid.atGoal()
        && Math.abs(headingError) < 0.3;
  }

  @Override
  public void resetDriveToPoseControllers() {
    Pose2d current = getPose2d();
    moveToPoseXAxisPid.reset(current.getX());
    moveToPoseYAxisPid.reset(current.getY());
  }

  protected void driveFieldOrientedLockedAngle(ChassisSpeeds speeds, Rotation2d targetHeading) {
    this.lastDesiredJoystickAngle = targetHeading.getRadians();
    this.targetHeadingDegrees = targetHeading.getDegrees();

    double omega = swerveDrive.getSwerveController().headingCalculate(
        swerveDrive.getOdometryHeading().getRadians(),
        targetHeading.getRadians());

    ChassisSpeeds locked = new ChassisSpeeds(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        omega);

    swerveDrive.driveFieldOriented(locked);
  }


  public void driveToNearestCoralStation() {
    Pose2d nearestCoralStationPose2D = this.getNearestCoralStationPose();
    this.driveToPose(nearestCoralStationPose2D);
  }


  private Pose2d getNearestCoralStationPose() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      if (getPose2d().getY() >= 4.0259) {
        return FieldElementsConst.CoralStations.RedAliance.CORAL_STATION_RIGHT_POSE_FOR_ROBOT;
      } else {
        return FieldElementsConst.CoralStations.RedAliance.CORAL_STATION_LEFT_POSE_FOR_ROBOT;
      }
    } else {
      if (getPose2d().getY() >= 4.0259) {
        return FieldElementsConst.CoralStations.BlueAliance.CORAL_STATION_LEFT_POSE_FOR_ROBOT;
      } else {
        return FieldElementsConst.CoralStations.BlueAliance.CORAL_STATION_RIGHT_POSE_FOR_ROBOT;
      }
    }
  }

  private Pose2d getNearestSupportPointPose() {
      boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
      return isRed
          ? FieldElementsConst.SupportPoints.RED_ALLIANCE_SUPPORT_POINT
          : FieldElementsConst.SupportPoints.BLUE_ALLIANCE_SUPPORT_POINT;
  }

  @Override
  public void driveToSupportPoint(){
    Pose2d supportPoint = this.getNearestSupportPointPose();
    this.driveToPose(supportPoint);
  }

  private void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose2d,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> {
            swerveDrive.drive(
              speeds,
              swerveDrive.kinematics.toSwerveModuleStates(speeds),
              feedforwards.linearForces()
            );
          },
          isSimulation() ?
            new PPHolonomicDriveController(SIM_TRANSLATION_PID.getPIDConsants(), SIM_ROTATION_PID.getPIDConsants()) :
            new PPHolonomicDriveController(REAL_TRANSLATION_PID.getPIDConsants(), REAL_ROTATION_PID.getPIDConsants()),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);

      PathPlannerLogging.setLogActivePathCallback(
          (activePath) -> {
            Logger.recordOutput(
                ACTIVE_TRACJECTORY_LOG_ENTRY,
                activePath.toArray(new Pose2d[activePath.size()]));
          });

      PathPlannerLogging.setLogTargetPoseCallback(
          (targetPose) -> {
            Logger.recordOutput(TRAJECTORY_SETPOINT_LOG_ENTRY, targetPose);
          });

    } catch (Exception e) {
      DriverStation.reportError(
          "Falha ao configurar o PathPlanner - autonomo desativado: " + e.getMessage(), e.getStackTrace());
    }
  }

  @Override
  public boolean inAllianceZone() {
        Pose2d pose = getPose2d();
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        && pose.getMeasureX().lt(ALLIANCE_ZONE)
                || DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        && pose.getMeasureX().gt(FIELD_LENGTH.minus(ALLIANCE_ZONE));
    }

  @Override
  public double getYaw() {
    return pigeon.getYaw().getValueAsDouble();
  }

  @Override
  public double getPitch() {
      return pigeon.getPitch().getValueAsDouble();
  }

  @Override
  public double getRoll() {
      return pigeon.getRoll().getValueAsDouble();
  }

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.currentPose2d = this.getPose2d();
    inputs.targetPose2d = this.targetPose != null ? this.targetPose : this.getPose2d();

    inputs.moduleStates = (DriverStation.isDisabled() || states == null) ? new SwerveModuleState[] {} : states;

    double[] encoderPos = new double[encoders.length];
    for (int i = 0; i < encoders.length; i++){
      encoderPos[i] = encoders[i].getPosition().getValueAsDouble();
    }

    inputs.currentCanCodersPosition = encoderPos;
    inputs.chassisSpeeds = getRobotRelativeSpeeds();
  }




  public void driveAroundPoint(Translation2d translation, double rotation, Translation2d centerOfRotation) {
    this.swerveDrive.drive(translation, rotation, true, false, centerOfRotation);
  }


  public Command pivotAroundTurretCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return run(() -> {
      double vx = Math.pow(x.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity();
      double vy = Math.pow(y.getAsDouble(), 3) * swerveDrive.getMaximumChassisVelocity();
      double rot = omega.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity();

      driveAroundPoint(new Translation2d(vx, vy), rot, TURRET_PIVOT);
      Logger.recordOutput("Swerve/CenterOfRotation", TURRET_PIVOT);
    });
  }




  public Command sysIdDriveCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive, 12.0, true),
        3.0, 5.0, 3.0);
  }


  public Command sysIdAngleCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new SysIdRoutine.Config(), this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  @Override
  public Command driveAnguladoCommand(DoubleSupplier X, DoubleSupplier Y,
                                      DoubleSupplier headingX, DoubleSupplier headingY) {
    return run(() -> {
      Translation2d scaled = SwerveMath.scaleTranslation(
          new Translation2d(X.getAsDouble(), Y.getAsDouble()), 0.8);

      ChassisSpeeds speeds = swerveDrive.swerveController.getTargetSpeeds(
          scaled.getX(), scaled.getY(),
          headingX.getAsDouble(), headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity());

          swerveDrive.driveFieldOriented(speeds);
    });
  }
}
