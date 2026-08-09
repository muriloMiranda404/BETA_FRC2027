package frc.frc_java9485.utils.Rebuilt;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.frc_java9485.constants.utils.FieldElementsConst.FieldMeansureds;

class ZonesTest {

    private static final Zones.PredictiveXBaseZone BOX =
            new Zones.PredictiveXBaseZone(2.0, 4.0, 1.0, 3.0);

    @Test
    void containsPointInsideAndOutside() {
        assertTrue(BOX.containsPoint(new Translation2d(3.0, 2.0)));
        assertFalse(BOX.containsPoint(new Translation2d(1.0, 2.0)));
        assertFalse(BOX.containsPoint(new Translation2d(3.0, 5.0)));
    }

    @Test
    void bordersCountAsInside() {
        assertTrue(BOX.containsPoint(new Translation2d(2.0, 1.0)));
        assertTrue(BOX.containsPoint(new Translation2d(4.0, 3.0)));
    }


    @Test
    void cornersAreNormalized() {
        Zones.BaseZone reversed = new Zones.BaseZone(4.0, 2.0, 3.0, 1.0);
        assertTrue(reversed.containsPoint(new Translation2d(3.0, 2.0)));
    }

    @Test
    void mirroredXReflectsAcrossTheFieldLength() {
        Zones.BaseZone mirrored = BOX.mirroredX();
        double length = FieldMeansureds.FIELD_LENGTH_METERS;

        assertTrue(mirrored.containsPoint(new Translation2d(length - 3.0, 2.0)));
        assertFalse(mirrored.containsPoint(new Translation2d(3.0, 2.0)));
    }

    @Test
    void mirroredYReflectsAcrossTheFieldWidth() {
        Zones.BaseZone mirrored = BOX.mirroredY();
        double width = FieldMeansureds.FIELD_WIDTH_METERS;

        assertTrue(mirrored.containsPoint(new Translation2d(3.0, width - 2.0)));
    }



    @Test
    void alreadyInsideCountsAsWillContain() {
        assertTrue(BOX.willContainPoint(new Translation2d(3.0, 2.0), new ChassisSpeeds(), Seconds.of(0.5)));
    }


    @Test
    void approachingFastEnoughWillContain() {
        Translation2d approaching = new Translation2d(1.0, 2.0);
        ChassisSpeeds towards = new ChassisSpeeds(4.0, 0.0, 0.0);

        assertTrue(BOX.willContainPoint(approaching, towards, Seconds.of(0.5)));
    }

    @Test
    void approachingTooSlowlyWillNotContain() {
        Translation2d approaching = new Translation2d(1.0, 2.0);
        ChassisSpeeds slow = new ChassisSpeeds(0.5, 0.0, 0.0);

        assertFalse(BOX.willContainPoint(approaching, slow, Seconds.of(0.5)));
    }

    @Test
    void drivingAwayWillNotContain() {
        Translation2d outside = new Translation2d(1.0, 2.0);
        ChassisSpeeds away = new ChassisSpeeds(-4.0, 0.0, 0.0);

        assertFalse(BOX.willContainPoint(outside, away, Seconds.of(0.5)));
    }


    @Test
    void wrongYBandIsNeverPredictedInside() {
        Translation2d offBand = new Translation2d(1.0, 10.0);
        ChassisSpeeds fast = new ChassisSpeeds(10.0, 0.0, 0.0);

        assertFalse(BOX.willContainPoint(offBand, fast, Seconds.of(1.0)));
    }

    @Test
    void cornersFormAClosedLoop() {
        Translation2d[] corners = BOX.getCorners();

        assertEquals(5, corners.length);
        assertEquals(corners[0], corners[4]);
    }

    @Test
    void allianceZonesCoverTheirOwnEndsOfTheField() {
        assertTrue(Zones.BLUE_ALLIANCE_ZONE.containsPoint(new Translation2d(1.0, 4.0)));
        assertFalse(Zones.BLUE_ALLIANCE_ZONE.containsPoint(
                new Translation2d(FieldMeansureds.FIELD_LENGTH_METERS - 1.0, 4.0)));

        assertTrue(Zones.RED_ALLIANCE_ZONE.containsPoint(
                new Translation2d(FieldMeansureds.FIELD_LENGTH_METERS - 1.0, 4.0)));


        Translation2d midfield = new Translation2d(FieldMeansureds.FIELD_LENGTH_METERS / 2.0, 4.0);
        assertFalse(Zones.BLUE_ALLIANCE_ZONE.containsPoint(midfield));
        assertFalse(Zones.RED_ALLIANCE_ZONE.containsPoint(midfield));
        assertTrue(Zones.NEUTRAL_ZONE.containsPoint(midfield));
    }
}
