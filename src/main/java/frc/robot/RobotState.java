package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.frc_java9485.constants.utils.FieldConsts;

import java.util.Optional;


public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private String gameSpecificMessage = null;


    private Pose2d fieldToRobot = Pose2d.kZero;
    private double lastTimestamp = Double.NEGATIVE_INFINITY;


    private ChassisSpeeds measuredFieldRelativeChassisSpeeds = new ChassisSpeeds();


    private Optional<Pose2d> latestVisionPose = Optional.empty();
    private double latestVisionTimestamp = 0.0;


    private double pitchDegrees = 0.0;
    private double rollDegrees = 0.0;

    private RobotState() {}


    public void addOdometryMeasurement(
            double timestamp, Pose2d pose, ChassisSpeeds fieldRelativeChassisSpeeds) {
        updateRobotPoseIfNewer(timestamp, pose);
        this.measuredFieldRelativeChassisSpeeds = fieldRelativeChassisSpeeds;
    }


    public void setRobotAttitude(double pitchDegrees, double rollDegrees) {
        this.pitchDegrees = pitchDegrees;
        this.rollDegrees = rollDegrees;
    }

    public double getPitchDegrees() {
        return pitchDegrees;
    }

    public double getRollDegrees() {
        return rollDegrees;
    }


    public double getYawRateRadPerSec() {
        return measuredFieldRelativeChassisSpeeds.omegaRadiansPerSecond;
    }


    public void addVisionObservation(double timestamp, Pose2d visionPose) {
        this.latestVisionPose = Optional.of(visionPose);
        this.latestVisionTimestamp = timestamp;
    }


    public Pose2d getFieldToRobotPose() {
        return fieldToRobot;
    }


    public ChassisSpeeds getMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds;
    }


    public Optional<Pose2d> getLatestVisionPose() {
        return latestVisionPose;
    }

    public double getLatestVisionTimestamp() {
        return latestVisionTimestamp;
    }


    public void setGameSpecificMessage(String gameSpecificMessage) {
        this.gameSpecificMessage = gameSpecificMessage;
    }

    public Optional<String> getGameSpecificMessage() {
        return Optional.ofNullable(gameSpecificMessage);
    }


    public boolean wonAuto() {
        return Optional.ofNullable(gameSpecificMessage)
                .map(message -> (FieldConsts.isBlueAlliance() && message.equals("B"))
                        || (!FieldConsts.isBlueAlliance() && message.equals("R")))
                .orElse(false);
    }

    private void updateRobotPoseIfNewer(double timestamp, Pose2d pose) {
        if (timestamp >= lastTimestamp) {
            lastTimestamp = timestamp;
            fieldToRobot = pose;
        }
    }
}
