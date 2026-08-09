package frc.robot.subsystems.vision;

import java.util.Optional;

import static frc.frc_java9485.constants.robot.VisionConsts.APRIL_TAG_FIELD_LAYOUT;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.frc_java9485.loggers.CustomBooleanLogger;
import frc.frc_java9485.loggers.CustomDoubleLogger;
import frc.frc_java9485.loggers.CustomPose2dLogger;
import frc.frc_java9485.loggers.CustomStringLogger;
import frc.robot.subsystems.swerve.StaticSwerve;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.io.PoseEstimator;

public class LimelightPoseEstimator implements PoseEstimator {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();


  double fieldLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength();

  double fieldWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

  double limitAngVelForUpdating;

  String limelightName;

  boolean only2TagsMeasurements = false;

  boolean useMegaTag1;

  private Boolean useVisionHeadingCorrection = false;

  private int[] tagFilter = null;
  private boolean tagFilterDirty = false;

  CustomStringLogger stateOfPoseUpdate;

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  CustomDoubleLogger distToTag;

  CustomDoubleLogger headingMegaTag2;

  CustomDoubleLogger stdDevXYLogger;

  CustomDoubleLogger stdDevThetaLogger;

  public LimelightPoseEstimator(String limelightName, boolean only2TagsMeasurements, boolean useMegaTag1,
      double limitAngVelForUpdating) {
    this.limelightName = limelightName;
    this.only2TagsMeasurements = only2TagsMeasurements;
    this.useMegaTag1 = useMegaTag1;
    this.limitAngVelForUpdating = limitAngVelForUpdating;
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/IsDetectingTags");
    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/NumberOfDetectedTags");
    this.headingMegaTag2 = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/Heading MegaTag2");
    this.distToTag = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/DistanceToTag");
    this.stateOfPoseUpdate = new CustomStringLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/StateOfPoseUpdate");
    this.stdDevXYLogger = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/stdDevXY");
    this.stdDevThetaLogger = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/stdDevTheta");
  }

  public LimelightPoseEstimator(String limelightName, boolean only2TagsMeasurements) {
    this(limelightName, only2TagsMeasurements, false, 3);
  }

  public LimelightPoseEstimator(String limelightName) {
    this(limelightName, false);
  }


  public void setTagFilter(int[] validIds) {
    this.tagFilter = (validIds == null || validIds.length == 0) ? null : validIds.clone();
    this.tagFilterDirty = true;
  }


  public void clearTagFilter() {
    setTagFilter(null);
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    try {
      applyTagFilterIfChanged();

      double measuredYawRateDegPerSec = Math.toDegrees(StaticSwerve.getMeasuredAngularVelocity());


      if (!useMegaTag1) {
        LimelightHelpers.SetRobotOrientation(
            this.limelightName, referencePose.getRotation().getDegrees(), measuredYawRateDegPerSec, 0, 0, 0, 0);
        this.headingMegaTag2.append(referencePose.getRotation().getDegrees());
      }

      if (!LimelightHelpers.getTV(this.limelightName)
          || Math.abs(StaticSwerve.getMeasuredAngularVelocity()) >= limitAngVelForUpdating) {
        this.isDetectingLogger.append(false);
        this.numberOfDetectedTagsLogger.append(0);
        this.stateOfPoseUpdate.append("WITHOUT_TARGET_OR_HIGH_ANGULAR_VELOCITY");
        return Optional.empty();
      }

      PoseEstimate limelightPoseEstimate;
      if (useMegaTag1) {
        limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);
        this.stateOfPoseUpdate.append("GETTING_MEGATAG_1");
      } else {
        limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.limelightName);
        this.stateOfPoseUpdate.append("GETTING_MEGATAG_2");
      }

      if (limelightPoseEstimate == null || limelightPoseEstimate.tagCount == 0) {
        this.isDetectingLogger.append(false);
        this.numberOfDetectedTagsLogger.append(0);
        this.stateOfPoseUpdate.append("REJECTED_BY_NO_TAGS");
        return Optional.empty();
      }

      PoseEstimation poseEstimation = convertPoseEstimate(limelightPoseEstimate);

      if (poseOutOfField(poseEstimation)) {
        this.stateOfPoseUpdate.append("REJECTED_BY_OUT_OF_THE_FIELD_ESTIMATION");
        return Optional.empty();
      }
      if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
        this.stateOfPoseUpdate.append("REJECTED_BY_ONLY_2_TAGS_MEASUREMENTS");
        return Optional.empty();
      }


      this.isDetectingLogger.append(true);
      this.detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
      this.numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
      this.distToTag.append(poseEstimation.distanceToTag);
      this.stdDevXYLogger.append(poseEstimation.visionStdDev.get(0, 0));
      this.stdDevThetaLogger.append(poseEstimation.visionStdDev.get(2, 0));

      return Optional.of(poseEstimation);
    } catch (Exception e) {
      this.stateOfPoseUpdate.append("EXCEPTION: " + e.getMessage());
      DriverStation.reportError(
          "LimelightPoseEstimator[" + this.limelightName + "] error: " + e.getMessage(), e.getStackTrace());
      return Optional.empty();
    }
  }


  private void applyTagFilterIfChanged() {
    if (!tagFilterDirty) {
      return;
    }
    LimelightHelpers.SetFiducialIDFiltersOverride(
        this.limelightName, tagFilter == null ? new int[0] : tagFilter);
    tagFilterDirty = false;
  }

  private boolean poseOutOfField(PoseEstimation pose2D) {
    double x = pose2D.estimatedPose.getX();
    double y = pose2D.estimatedPose.getY();
    return (x <= 0 || x >= fieldLength) || (y <= 0 || y >= fieldWidth);
  }

  private PoseEstimation convertPoseEstimate(PoseEstimate limelightPoseEstimate) {
    double stdDevXY = 0.04 * Math.pow(limelightPoseEstimate.avgTagDist, 2)
        / limelightPoseEstimate.tagCount;
    double stdDevTheta = useVisionHeadingCorrection
        ? 0.04 * Math.pow(limelightPoseEstimate.avgTagDist, 2)
            / limelightPoseEstimate.tagCount
        : Double.POSITIVE_INFINITY;
    return new PoseEstimation(new Pose3d(limelightPoseEstimate.pose), limelightPoseEstimate.timestampSeconds,
        limelightPoseEstimate.tagCount, limelightPoseEstimate.avgTagDist, VecBuilder.fill(stdDevXY,
            stdDevXY, stdDevTheta));
  }

  public String getEstimatorName() {
    return this.limelightName;
  }
}
