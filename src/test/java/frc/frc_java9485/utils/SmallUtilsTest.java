package frc.frc_java9485.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.frc_java9485.utils.calc.ClosedInterval;
import frc.frc_java9485.utils.control.DelayedBoolean;
import frc.frc_java9485.utils.geometry.PoseErrorTolerance;


class SmallUtilsTest {



    @Test
    void delayedBooleanWaitsForTheDelay() {
        DelayedBoolean latch = new DelayedBoolean(0.0, 0.5);

        assertFalse(latch.update(0.0, true));
        assertFalse(latch.update(0.4, true), "must not fire before the delay elapses");
        assertTrue(latch.update(0.6, true), "should fire once the delay has passed");
    }

    @Test
    void delayedBooleanDropsImmediatelyAndRestartsTheDelay() {
        DelayedBoolean latch = new DelayedBoolean(0.0, 0.5);

        latch.update(0.0, true);
        assertTrue(latch.update(1.0, true));


        assertFalse(latch.update(1.1, false));


        assertFalse(latch.update(1.2, true));
        assertFalse(latch.update(1.5, true));
        assertTrue(latch.update(1.8, true));
    }

    @Test
    void delayedBooleanResetForcesNotYet() {
        DelayedBoolean latch = new DelayedBoolean(0.0, 0.2);

        latch.update(0.0, true);
        assertTrue(latch.update(1.0, true));

        latch.reset(1.0);
        assertFalse(latch.update(1.1, true));
    }



    @Test
    void closedIntervalBasics() {
        ClosedInterval interval = new ClosedInterval(5, 9);

        assertEquals(5, interval.getStart());
        assertEquals(9, interval.getEnd());
        assertEquals(4, interval.getLength());
        assertEquals(5, interval.size());
        assertTrue(interval.contains(5));
        assertTrue(interval.contains(9));
        assertFalse(interval.contains(10));
    }

    @Test
    void closedIntervalIndexIsRelativeToStart() {
        ClosedInterval interval = new ClosedInterval(5, 9);

        assertEquals(5, interval.getIndex(0));
        assertEquals(9, interval.getIndex(4));
        assertThrows(IndexOutOfBoundsException.class, () -> interval.getIndex(5));
        assertThrows(IndexOutOfBoundsException.class, () -> interval.getIndex(-1));
    }


    @Test
    void closedIntervalSubRangeIsClamped() {
        ClosedInterval interval = new ClosedInterval(5, 9);

        assertEquals(new ClosedInterval(5, 7), interval.getFromIndexRange(0, 2));
        assertEquals(new ClosedInterval(5, 9), interval.getFromIndexRange(0, 100));
        assertEquals(new ClosedInterval(5, 9), interval.getFromIndexRange(-100, 100));
    }

    @Test
    void closedIntervalDetectsOverlap() {
        ClosedInterval a = new ClosedInterval(0, 4);

        assertTrue(a.collides(new ClosedInterval(4, 8)), "touching at one index is an overlap");
        assertFalse(a.collides(new ClosedInterval(5, 8)));
        assertTrue(a.collides(new ClosedInterval(1, 2)));
    }

    @Test
    void closedIntervalEqualityAndHashing() {
        assertEquals(new ClosedInterval(2, 6), new ClosedInterval(2, 6));
        assertEquals(new ClosedInterval(2, 6).hashCode(), new ClosedInterval(2, 6).hashCode());
        assertNotEquals(new ClosedInterval(2, 6), new ClosedInterval(2, 7));
    }

    @Test
    void closedIntervalRejectsBackwardsRange() {
        assertThrows(IllegalArgumentException.class, () -> new ClosedInterval(9, 5));
    }



    @Test
    void poseToleranceAcceptsWithinBothBounds() {
        PoseErrorTolerance tolerance = PoseErrorTolerance.of(0.05, 2.0);
        Pose2d expected = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));

        assertTrue(tolerance.atPose(expected, new Pose2d(1.02, 2.02, Rotation2d.fromDegrees(31.0))));
    }

    @Test
    void poseToleranceRejectsEitherBoundAlone() {
        PoseErrorTolerance tolerance = PoseErrorTolerance.of(0.05, 2.0);
        Pose2d expected = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));


        assertFalse(tolerance.atPose(expected, new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(30.0))));

        assertFalse(tolerance.atPose(expected, new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45.0))));
    }


    @Test
    void poseToleranceWrapsRotationAcrossPlusMinus180() {
        PoseErrorTolerance tolerance = PoseErrorTolerance.of(0.05, 5.0);
        Pose2d expected = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(179.0));

        assertTrue(tolerance.atRotation(expected, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-179.0))));
        assertTrue(tolerance.atPose(expected, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-179.0))));
    }

    @Test
    void poseToleranceSplitsTranslationAndRotationChecks() {
        PoseErrorTolerance tolerance = PoseErrorTolerance.of(0.05, 2.0);
        Pose2d expected = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
        Pose2d rotatedOnly = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0));

        assertTrue(tolerance.atTranslation(expected, rotatedOnly));
        assertFalse(tolerance.atRotation(expected, rotatedOnly));
    }
}
