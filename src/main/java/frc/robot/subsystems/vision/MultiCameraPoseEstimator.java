package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.frc_java9485.loggers.CustomDoubleLogger;
import frc.frc_java9485.loggers.CustomPose2dLogger;
import frc.frc_java9485.loggers.CustomStringLogger;
import frc.robot.subsystems.vision.io.PoseEstimator;


public class MultiCameraPoseEstimator implements PoseEstimator {


    private static final Comparator<PoseEstimation> BEST_FIRST =
            Comparator.comparingInt((PoseEstimation e) -> e.numberOfTargetsUsed).reversed()
                    .thenComparingDouble(e -> e.distanceToTag);

    private final List<PoseEstimator> estimators;
    private final String name;

    private final CustomPose2dLogger detectedPoseLogger;
    private final CustomDoubleLogger numberOfDetectedTagsLogger;
    private final CustomDoubleLogger distanceToTagLogger;
    private final CustomStringLogger chosenCameraLogger;

    public MultiCameraPoseEstimator(List<PoseEstimator> estimators, String name) {
        this.estimators = List.copyOf(estimators);
        this.name = name;

        String base = "/Vision/MultiCameraPoseEstimator/" + name + "/";
        this.detectedPoseLogger = new CustomPose2dLogger(base + "DetectedPose");
        this.numberOfDetectedTagsLogger = new CustomDoubleLogger(base + "NumberOfDetectedTags");
        this.distanceToTagLogger = new CustomDoubleLogger(base + "DistanceToTag");
        this.chosenCameraLogger = new CustomStringLogger(base + "ChosenCamera");
    }

    @Override
    public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
        List<PoseEstimation> candidates = new ArrayList<>();
        List<String> candidateNames = new ArrayList<>();

        for (PoseEstimator estimator : estimators) {

            Optional<PoseEstimation> estimate = estimator.getEstimatedPose(referencePose);
            if (estimate.isPresent()) {
                candidates.add(estimate.get());
                candidateNames.add(estimator.getEstimatorName());
            }
        }

        if (candidates.isEmpty()) {
            numberOfDetectedTagsLogger.append(0);
            chosenCameraLogger.append("NONE");
            return Optional.empty();
        }

        int bestIndex = 0;
        for (int i = 1; i < candidates.size(); i++) {
            if (BEST_FIRST.compare(candidates.get(i), candidates.get(bestIndex)) < 0) {
                bestIndex = i;
            }
        }

        PoseEstimation best = candidates.get(bestIndex);
        detectedPoseLogger.appendRadians(best.estimatedPose.toPose2d());
        numberOfDetectedTagsLogger.append(best.numberOfTargetsUsed);
        distanceToTagLogger.append(best.distanceToTag);
        chosenCameraLogger.append(candidateNames.get(bestIndex));

        return Optional.of(best);
    }

    @Override
    public String getEstimatorName() {
        return name;
    }
}
