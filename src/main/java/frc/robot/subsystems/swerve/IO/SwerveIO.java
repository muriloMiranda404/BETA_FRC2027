package frc.robot.subsystems.swerve.IO;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;

import swervelib.simulation.ironmaple.simulation.drivesims.GyroSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public static class SwerveInputs{
    public Pose2d currentPose2d = new Pose2d();
    public Pose2d targetPose2d = new Pose2d();
    public SwerveModuleState[] moduleStates = {};
    public double[] currentCanCodersPosition = {0, 0, 0, 0};
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    public double yaw = 0;
    public double pitch = 0;
    public double roll = 0;
  }

  public Pose2d getPose2d();

  public Rotation2d getHeading2d();
  public Rotation3d getHeading3d();

  default void resetOdometry(Pose2d pose){};

  default void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds){};
  default void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs){};
  default void driveFieldOriented(ChassisSpeeds speed){};
  default void driveToSupportPoint(){};

  default void updateInputs(SwerveInputs inputs){};
  default void drive(Translation2d translation2d, double rotation, boolean fieldOriented){};
  default void lock(){};
  default void resetDriveToPoseControllers(){};

  public Pigeon2 getPigeon();
  public GyroSimulation getGyroSimulation();

  public SwerveDriveSimulation getSimulation();

  public ChassisSpeeds getRobotRelativeSpeeds();

  public boolean inAllianceZone();

  public boolean atTargetPose();

  public Command getAutonomousCommand(String path, boolean altern);

  public Command driveCommand(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, boolean fieldOriented);

  public double getYaw();

  public double getPitch();

  public double getRoll();

  public Command driveAnguladoCommand(DoubleSupplier X, DoubleSupplier Y,
                                      DoubleSupplier headingX, DoubleSupplier headingY);

  public double getAngularVelocity();
}
