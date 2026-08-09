package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


public class PoseEstimation {

  public final Pose3d estimatedPose;


  public final double timestampSeconds;


  public int numberOfTargetsUsed;


  public final double distanceToTag;

  public Matrix<N3, N1> visionStdDev = VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY);


  public PoseEstimation(
      Pose3d estimatedPose,
      double timestampSeconds,
      int numberOfTargetsUsed, double distanceToTag, Matrix<N3, N1> visionStdDev) {
    this.estimatedPose = estimatedPose;
    this.timestampSeconds = timestampSeconds;
    this.numberOfTargetsUsed = numberOfTargetsUsed;
    this.distanceToTag = distanceToTag;
    this.visionStdDev = visionStdDev;
  }

  public PoseEstimation(
      Pose3d estimatedPose,
      double timestampSeconds,
      int numberOfTargetsUsed, double distanceToTag) {
    this.estimatedPose = estimatedPose;
    this.timestampSeconds = timestampSeconds;
    this.numberOfTargetsUsed = numberOfTargetsUsed;
    this.distanceToTag = distanceToTag;
  }

  public void changeVisionStdDev(Matrix<N3, N1> visionStdDev) {
    this.visionStdDev = visionStdDev;
  }
}
