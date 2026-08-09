package frc.robot.subsystems.mechanism.shooter;

import edu.wpi.first.math.MathUtil;
import frc.frc_java9485.constants.mechanisms.shooter.ShotVerifierConsts;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator.ShotSolution;


public class ShotVerifier {

    private ShotVerifier() {}


    public enum Rejection {
        NONE,
        NOT_AIMED,
        TURNING_TOO_FAST,
        ROBOT_TIPPED,
        TOO_FAR,
        TOO_CLOSE,
        NO_SOLUTION
    }


    public record Verification(boolean verified, Rejection rejection) {
        public static final Verification OK = new Verification(true, Rejection.NONE);

        public static Verification rejected(Rejection reason) {
            return new Verification(false, reason);
        }
    }


    public static Verification verifyHubShot(
            ShotSolution solution,
            double headingDeg,
            double yawRateRadPerSec,
            double pitchDeg,
            double rollDeg) {
        return verify(
                solution,
                headingDeg,
                yawRateRadPerSec,
                pitchDeg,
                rollDeg,
                ShotVerifierConsts.ACCEPTABLE_HUB_ANGLE_ERROR_DEG);
    }


    public static Verification verifyPassShot(
            ShotSolution solution,
            double headingDeg,
            double yawRateRadPerSec,
            double pitchDeg,
            double rollDeg) {
        return verify(
                solution,
                headingDeg,
                yawRateRadPerSec,
                pitchDeg,
                rollDeg,
                ShotVerifierConsts.ACCEPTABLE_PASS_ANGLE_ERROR_DEG);
    }

    private static Verification verify(
            ShotSolution solution,
            double headingDeg,
            double yawRateRadPerSec,
            double pitchDeg,
            double rollDeg,
            double angleToleranceDeg) {

        if (solution == null) {
            return Verification.rejected(Rejection.NO_SOLUTION);
        }
        if (isTipped(pitchDeg, rollDeg)) {
            return Verification.rejected(Rejection.ROBOT_TIPPED);
        }
        if (solution.compensatedDistanceM() > ShotVerifierConsts.MAX_SHOT_DISTANCE_M) {
            return Verification.rejected(Rejection.TOO_FAR);
        }
        if (solution.compensatedDistanceM() < ShotVerifierConsts.MIN_SHOT_DISTANCE_M) {
            return Verification.rejected(Rejection.TOO_CLOSE);
        }
        if (Math.abs(yawRateRadPerSec) > ShotVerifierConsts.MAX_YAW_RATE_RAD_PER_SEC) {
            return Verification.rejected(Rejection.TURNING_TOO_FAST);
        }
        if (!isAimed(solution, headingDeg, angleToleranceDeg)) {
            return Verification.rejected(Rejection.NOT_AIMED);
        }
        return Verification.OK;
    }


    public static boolean isAimed(ShotSolution solution, double headingDeg, double toleranceDeg) {
        return Math.abs(headingErrorDeg(solution, headingDeg)) <= toleranceDeg;
    }


    public static double headingErrorDeg(ShotSolution solution, double headingDeg) {
        return MathUtil.inputModulus(solution.fieldHeadingDeg() - headingDeg, -180.0, 180.0);
    }


    public static boolean isTipped(double pitchDeg, double rollDeg) {

        if (!Double.isFinite(pitchDeg) || !Double.isFinite(rollDeg)) {
            return false;
        }
        double tolerance = ShotVerifierConsts.ACCEPTABLE_TILT_ERROR_DEG;
        return Math.abs(pitchDeg) > tolerance || Math.abs(rollDeg) > tolerance;
    }


    public static boolean isInRange(double distanceMeters) {
        return distanceMeters >= ShotVerifierConsts.MIN_SHOT_DISTANCE_M
                && distanceMeters <= ShotVerifierConsts.MAX_SHOT_DISTANCE_M;
    }
}
