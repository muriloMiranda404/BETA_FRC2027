package frc.robot.commands.swerveUtils.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.frc_java9485.constants.mechanisms.shooter.ShotVerifierConsts;


class DriveIntoShotRangeTest {

    private static final Translation2d HUB = new Translation2d(4.0, 4.0);

    private static final double MIN = ShotVerifierConsts.MIN_SHOT_DISTANCE_M;
    private static final double MAX = ShotVerifierConsts.MAX_SHOT_DISTANCE_M;
    private static final double MARGIN = DriveIntoShotRange.RANGE_MARGIN_M;

    private static Pose2d nearest(Translation2d robot) {
        return DriveIntoShotRange.nearestShootingPose(robot, HUB, MIN, MAX, MARGIN);
    }

    @Test
    void alreadyInRangeStaysPut() {
        Translation2d robot = new Translation2d(4.0 + 3.0, 4.0);
        Pose2d goal = nearest(robot);

        assertEquals(robot.getX(), goal.getX(), 1e-6);
        assertEquals(robot.getY(), goal.getY(), 1e-6);
    }

    @Test
    void tooFarIsPulledOntoTheOuterBound() {
        Translation2d robot = new Translation2d(4.0 + 12.0, 4.0);
        Pose2d goal = nearest(robot);

        assertEquals(MAX - MARGIN, goal.getTranslation().getDistance(HUB), 1e-6);
    }

    @Test
    void tooCloseIsPushedOutToTheInnerBound() {
        Translation2d robot = new Translation2d(4.0 + 0.2, 4.0);
        Pose2d goal = nearest(robot);

        assertEquals(MIN + MARGIN, goal.getTranslation().getDistance(HUB), 1e-6);
    }


    @Test
    void bearingFromTheHubIsPreserved() {
        Translation2d robot = new Translation2d(4.0 + 8.0, 4.0 + 8.0);
        Pose2d goal = nearest(robot);

        double robotBearing = Math.atan2(robot.getY() - HUB.getY(), robot.getX() - HUB.getX());
        double goalBearing = Math.atan2(goal.getY() - HUB.getY(), goal.getX() - HUB.getX());

        assertEquals(robotBearing, goalBearing, 1e-6);
    }

    @Test
    void goalFacesTheHub() {
        Translation2d robot = new Translation2d(4.0 + 10.0, 4.0);
        Pose2d goal = nearest(robot);


        assertEquals(180.0, Math.abs(goal.getRotation().getDegrees()), 1e-6);
    }


    @Test
    void degenerateCaseOnTheHubStillProducesAValidPose() {
        Pose2d goal = nearest(HUB);

        assertEquals(MIN + MARGIN, goal.getTranslation().getDistance(HUB), 1e-6);
        assertTrue(Double.isFinite(goal.getRotation().getRadians()));
    }


    @Test
    void oversizedMarginFallsBackToTheBandMidpoint() {
        Pose2d goal = DriveIntoShotRange.nearestShootingPose(
                new Translation2d(4.0 + 20.0, 4.0), HUB, MIN, MAX, 100.0);

        assertEquals((MIN + MAX) / 2.0, goal.getTranslation().getDistance(HUB), 1e-6);
    }

    @Test
    void isInRangeMatchesTheVerifierBand() {
        assertTrue(DriveIntoShotRange.isInRange(new Translation2d(4.0 + 3.0, 4.0), HUB));
        assertFalse(DriveIntoShotRange.isInRange(new Translation2d(4.0 + 20.0, 4.0), HUB));
        assertFalse(DriveIntoShotRange.isInRange(new Translation2d(4.0 + 0.1, 4.0), HUB));
    }
}
