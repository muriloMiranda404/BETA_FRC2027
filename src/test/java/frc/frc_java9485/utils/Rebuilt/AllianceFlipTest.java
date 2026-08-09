package frc.frc_java9485.utils.Rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class AllianceFlipTest {
    private static final double DELTA = 1e-6;

    @Test
    void flipTranslationTwice_returnsOriginal() {
        Translation2d original = new Translation2d(2.0, 3.0);
        Translation2d twice =
            AllianceFlip.flipTranslation2dToRed(AllianceFlip.flipTranslation2dToRed(original));
        assertEquals(original.getX(), twice.getX(), DELTA);
        assertEquals(original.getY(), twice.getY(), DELTA);
    }

    @Test
    void flipTranslation_keepsYCoordinate() {
        Translation2d original = new Translation2d(2.0, 3.0);
        Translation2d flipped = AllianceFlip.flipTranslation2dToRed(original);
        assertEquals(original.getY(), flipped.getY(), DELTA);
    }

    @Test
    void flipPose_rotatesBy180AndKeepsY() {
        Pose2d original = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(30));
        Pose2d flipped = AllianceFlip.flipPose2dToRed(original);
        assertEquals(
            original.getRotation().plus(Rotation2d.k180deg).getRadians(),
            flipped.getRotation().getRadians(),
            DELTA);
        assertEquals(original.getY(), flipped.getY(), DELTA);
    }
}
