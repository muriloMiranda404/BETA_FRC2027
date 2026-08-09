package frc.robot.subsystems.vision;

import static frc.frc_java9485.constants.robot.RobotConsts.isSimulation;
import static frc.frc_java9485.constants.robot.VisionConsts.APRIL_TAG_FIELD_LAYOUT;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.frc_java9485.loggers.CustomBooleanLogger;
import frc.frc_java9485.loggers.CustomDoubleLogger;
import frc.frc_java9485.loggers.CustomPose2dLogger;
import frc.robot.subsystems.vision.io.PoseEstimator;


public class PhotonVisionPoseEstimator implements PoseEstimator {

  private static final double SINGLE_TAG_MAX_DISTANCE = 4.0;

  private static final double SINGLE_TAG_BASE_STD_DEV = 2.0;
  private static final double MULTI_TAG_BASE_STD_DEV = 0.5;

  private final String cameraName;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  private final double fieldLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength();
  private final double fieldWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  private final CustomBooleanLogger isDetectingLogger;
  private final CustomPose2dLogger detectedPoseLogger;
  private final CustomDoubleLogger numberOfDetectedTagsLogger;
  private final CustomDoubleLogger distToTagLogger;
  private final CustomDoubleLogger stdDevXYLogger;

  public PhotonVisionPoseEstimator(
      String cameraName, Transform3d robotToCamera, SimCameraProperties simProps) {
    this.cameraName = cameraName;
    this.camera = new PhotonCamera(cameraName);

    this.poseEstimator = new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, robotToCamera);

    String base = "/Vision/PhotonVisionPoseEstimator/" + cameraName + "/";
    this.isDetectingLogger = new CustomBooleanLogger(base + "IsDetectingTags");
    this.detectedPoseLogger = new CustomPose2dLogger(base + "DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(base + "NumberOfDetectedTags");
    this.distToTagLogger = new CustomDoubleLogger(base + "DistanceToTag");
    this.stdDevXYLogger = new CustomDoubleLogger(base + "stdDevXY");

    if (isSimulation() && simProps != null) {
      visionSim = new VisionSystemSim(cameraName);
      visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
      cameraSim = new PhotonCameraSim(camera, simProps);
      visionSim.addCamera(cameraSim, robotToCamera);
      cameraSim.enableDrawWireframe(true);
    }
  }

  @Override
  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {

    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), referencePose.getRotation());

    Optional<PoseEstimation> latest = Optional.empty();


    for (PhotonPipelineResult frame : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> estimate = estimate(frame);
      if (estimate.isEmpty()) continue;
      Optional<PoseEstimation> converted = convert(estimate.get());
      if (converted.isPresent()) latest = converted;
    }

    if (latest.isEmpty()) {
      isDetectingLogger.append(false);
      numberOfDetectedTagsLogger.append(0);
    } else {
      PoseEstimation pe = latest.get();
      isDetectingLogger.append(true);
      detectedPoseLogger.appendRadians(pe.estimatedPose.toPose2d());
      numberOfDetectedTagsLogger.append(pe.numberOfTargetsUsed);
      distToTagLogger.append(pe.distanceToTag);
      stdDevXYLogger.append(pe.visionStdDev.get(0, 0));
    }
    return latest;
  }


  private Optional<EstimatedRobotPose> estimate(PhotonPipelineResult frame) {
    if (frame.multitagResult.isPresent()) {
      return poseEstimator.estimateCoprocMultiTagPose(frame);
    }
    return poseEstimator.estimateLowestAmbiguityPose(frame);
  }

  private Optional<PoseEstimation> convert(EstimatedRobotPose estimate) {
    List<PhotonTrackedTarget> targets = estimate.targetsUsed;
    Pose3d pose = estimate.estimatedPose;

    int numTags = 0;
    double avgDist = 0;
    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist += tagPose.get().getTranslation().getDistance(pose.getTranslation());
    }
    if (numTags == 0) return Optional.empty();
    avgDist /= numTags;

    if (poseOutOfField(pose)) return Optional.empty();
    if (numTags == 1 && avgDist > SINGLE_TAG_MAX_DISTANCE) return Optional.empty();

    double baseStdDev = numTags > 1 ? MULTI_TAG_BASE_STD_DEV : SINGLE_TAG_BASE_STD_DEV;
    double xyStdDev = baseStdDev * (1 + (avgDist * avgDist / 30.0));


    return Optional.of(new PoseEstimation(
        pose, estimate.timestampSeconds, numTags, avgDist,
        VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY)));
  }

  private boolean poseOutOfField(Pose3d pose) {
    double x = pose.getX();
    double y = pose.getY();
    return x <= 0 || x >= fieldLength || y <= 0 || y >= fieldWidth;
  }


  public void updateSim(Pose2d robotPoseGroundTruth) {
    if (visionSim != null) visionSim.update(robotPoseGroundTruth);
  }

  @Override
  public String getEstimatorName() {
    return cameraName;
  }
}
