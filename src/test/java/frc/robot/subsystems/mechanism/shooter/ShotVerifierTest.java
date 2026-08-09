package frc.robot.subsystems.mechanism.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.frc_java9485.constants.mechanisms.shooter.ShotVerifierConsts;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator.ShotSolution;
import frc.robot.subsystems.mechanism.shooter.ShotVerifier.Rejection;


class ShotVerifierTest {


    private static ShotSolution solutionAt(double fieldHeadingDeg, double distanceM) {
        return new ShotSolution(fieldHeadingDeg, 0.0, 1.5, 2700.0, distanceM, 0.6, true);
    }

    private static ShotSolution goodSolution() {
        return solutionAt(0.0, 3.0);
    }

    @Test
    void aimedLevelAndStillIsVerified() {
        var result = ShotVerifier.verifyHubShot(goodSolution(), 0.0, 0.0, 0.0, 0.0);

        assertTrue(result.verified());
        assertEquals(Rejection.NONE, result.rejection());
    }

    @Test
    void missingSolutionIsRejected() {
        var result = ShotVerifier.verifyHubShot(null, 0.0, 0.0, 0.0, 0.0);

        assertFalse(result.verified());
        assertEquals(Rejection.NO_SOLUTION, result.rejection());
    }

    @Test
    void headingErrorBeyondToleranceIsRejected() {
        double justOver = ShotVerifierConsts.ACCEPTABLE_HUB_ANGLE_ERROR_DEG + 0.5;
        var result = ShotVerifier.verifyHubShot(goodSolution(), justOver, 0.0, 0.0, 0.0);

        assertFalse(result.verified());
        assertEquals(Rejection.NOT_AIMED, result.rejection());
    }


    @Test
    void passToleratesLargerHeadingError() {
        double error = ShotVerifierConsts.ACCEPTABLE_HUB_ANGLE_ERROR_DEG + 0.5;

        assertFalse(ShotVerifier.verifyHubShot(goodSolution(), error, 0.0, 0.0, 0.0).verified());
        assertTrue(ShotVerifier.verifyPassShot(goodSolution(), error, 0.0, 0.0, 0.0).verified());
    }


    @Test
    void spinningRobotIsRejectedEvenWhenAimed() {
        double fastYaw = ShotVerifierConsts.MAX_YAW_RATE_RAD_PER_SEC + 0.1;
        var result = ShotVerifier.verifyHubShot(goodSolution(), 0.0, fastYaw, 0.0, 0.0);

        assertFalse(result.verified());
        assertEquals(Rejection.TURNING_TOO_FAST, result.rejection());
    }

    @Test
    void tippedRobotIsRejected() {
        double tilt = ShotVerifierConsts.ACCEPTABLE_TILT_ERROR_DEG + 1.0;

        assertEquals(Rejection.ROBOT_TIPPED,
                ShotVerifier.verifyHubShot(goodSolution(), 0.0, 0.0, tilt, 0.0).rejection());
        assertEquals(Rejection.ROBOT_TIPPED,
                ShotVerifier.verifyHubShot(goodSolution(), 0.0, 0.0, 0.0, -tilt).rejection());
    }


    @Test
    void nonFiniteAttitudeIsTreatedAsLevel() {
        assertFalse(ShotVerifier.isTipped(Double.NaN, Double.NaN));
        assertTrue(ShotVerifier.verifyHubShot(goodSolution(), 0.0, 0.0, Double.NaN, Double.NaN).verified());
    }

    @Test
    void outOfRangeDistancesAreRejected() {
        ShotSolution tooFar = solutionAt(0.0, ShotVerifierConsts.MAX_SHOT_DISTANCE_M + 1.0);
        ShotSolution tooClose = solutionAt(0.0, ShotVerifierConsts.MIN_SHOT_DISTANCE_M - 0.1);

        assertEquals(Rejection.TOO_FAR, ShotVerifier.verifyHubShot(tooFar, 0.0, 0.0, 0.0, 0.0).rejection());
        assertEquals(Rejection.TOO_CLOSE, ShotVerifier.verifyHubShot(tooClose, 0.0, 0.0, 0.0, 0.0).rejection());
        assertFalse(ShotVerifier.isInRange(ShotVerifierConsts.MAX_SHOT_DISTANCE_M + 1.0));
        assertTrue(ShotVerifier.isInRange(3.0));
    }


    @Test
    void headingErrorWrapsAcrossPlusMinus180() {
        ShotSolution nearWrap = solutionAt(179.0, 3.0);


        assertEquals(-2.0, ShotVerifier.headingErrorDeg(nearWrap, -179.0), 1e-9);
        assertTrue(ShotVerifier.isAimed(nearWrap, 179.5, 1.0));
    }
}
