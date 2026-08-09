package frc.robot.subsystems.vision;

import static frc.frc_java9485.constants.robot.VisionConsts.LIMELIGHT_CAMERA_NAME;
import static frc.frc_java9485.constants.robot.VisionConsts.RASPBERRY_CAMERA_NAME;
import static frc.frc_java9485.constants.robot.VisionConsts.RASPBERRY_CAMERA_PROPS;
import static frc.frc_java9485.constants.robot.VisionConsts.RASPBERRY_ENABLED;
import static frc.frc_java9485.constants.robot.VisionConsts.RASPBERRY_ROBOT_TO_CAMERA;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.io.PoseEstimator;


public class VisionSubsystem extends SubsystemBase {
  private final SwerveSubsystem swerve;
  private final List<PoseEstimator> poseEstimators = new ArrayList<>();

  private final PhotonVisionPoseEstimator raspberryEstimator;

  public VisionSubsystem(SwerveSubsystem swerve) {
    this.swerve = swerve;


    if (RASPBERRY_ENABLED) {
      raspberryEstimator = new PhotonVisionPoseEstimator(
          RASPBERRY_CAMERA_NAME, RASPBERRY_ROBOT_TO_CAMERA, RASPBERRY_CAMERA_PROPS);
      poseEstimators.add(raspberryEstimator);
    } else {
      raspberryEstimator = null;
    }


    poseEstimators.add(new LimelightPoseEstimator(LIMELIGHT_CAMERA_NAME));
  }

  @Override
  public void periodic() {
    Pose2d referencePose = swerve.getPose2d();


    for (PoseEstimator estimator : poseEstimators) {
      estimator.getEstimatedPose(referencePose).ifPresent(estimation -> {
        Pose2d visionPose = estimation.estimatedPose.toPose2d();
        swerve.addVisionMeasurement(visionPose, estimation.timestampSeconds, estimation.visionStdDev);
        RobotState.getInstance().addVisionObservation(estimation.timestampSeconds, visionPose);
      });
    }
  }

  @Override
  public void simulationPeriodic() {

    if (raspberryEstimator != null) {
      raspberryEstimator.updateSim(swerve.getSimulation().getSimulatedDriveTrainPose());
    }
  }
}
