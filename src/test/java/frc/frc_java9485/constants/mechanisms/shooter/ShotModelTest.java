package frc.frc_java9485.constants.mechanisms.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts.ShotModel;
import org.junit.jupiter.api.Test;

class ShotModelTest {
    private static final double DELTA = 1e-6;

    @Test
    void rpm_returnsExactCalibrationPoints() {
        assertEquals(2200.0, ShotModel.DISTANCE_TO_RPM.get(1.0), DELTA);
        assertEquals(2500.0, ShotModel.DISTANCE_TO_RPM.get(2.5), DELTA);
        assertEquals(3400.0, ShotModel.DISTANCE_TO_RPM.get(6.0), DELTA);
    }

    @Test
    void rpm_interpolatesLinearlyBetweenPoints() {

        assertEquals(2350.0, ShotModel.DISTANCE_TO_RPM.get(1.75), DELTA);
    }

    @Test
    void rpm_clampsToEndpointsOutsideRange() {
        assertEquals(2200.0, ShotModel.DISTANCE_TO_RPM.get(0.0), DELTA);
        assertEquals(3400.0, ShotModel.DISTANCE_TO_RPM.get(50.0), DELTA);
    }

    @Test
    void hoodPosition_returnsExactCalibrationPoints() {
        assertEquals(0.5, ShotModel.DISTANCE_TO_HOOD_POSITION.get(1.0), DELTA);
        assertEquals(3.5, ShotModel.DISTANCE_TO_HOOD_POSITION.get(6.0), DELTA);
    }
}
