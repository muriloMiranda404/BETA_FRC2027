package frc.robot.subsystems.vision.io;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.PoseEstimation;

public interface PoseEstimator {

  public String getEstimatorName();

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose);

}
