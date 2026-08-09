package frc.frc_java9485.constants.mechanisms.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts.Setpoint;
import org.junit.jupiter.api.Test;


class TurretUnitsTest {
    private static final double DELTA = 1e-9;

    @Test
    void conversion_roundTrips() {
        assertEquals(37.5, Setpoint.motorRotationsToDegrees(Setpoint.degreesToMotorRotations(37.5)), 1e-9);
        assertEquals(-12.25, Setpoint.motorRotationsToDegrees(Setpoint.degreesToMotorRotations(-12.25)), 1e-9);
    }

    @Test
    void degreeLimitsMatchTheMotorRotationLimits() {
        assertEquals(
            Setpoint.MAX_TURN_POSITION, Setpoint.degreesToMotorRotations(Setpoint.MAX_TURN_ANGLE_DEG), DELTA);
        assertEquals(
            Setpoint.MIN_TURN_POSITION, Setpoint.degreesToMotorRotations(Setpoint.MIN_TURN_ANGLE_DEG), DELTA);
    }


    @Test
    void oneTurretDegreeIsMuchLessThanOneMotorRotation() {
        assertTrue(Setpoint.degreesToMotorRotations(1.0) > 0.0);
        assertTrue(Setpoint.degreesToMotorRotations(1.0) < 1.0);
        assertEquals(
            TurretConsts.Config.TURRET_REDUCTION / 360.0, Setpoint.degreesToMotorRotations(1.0), DELTA);
    }

    @Test
    void travelLimitsAreAPlausibleTurretRange() {

        assertTrue(Setpoint.MAX_TURN_ANGLE_DEG > 60.0 && Setpoint.MAX_TURN_ANGLE_DEG < 180.0,
            "unexpected max travel: " + Setpoint.MAX_TURN_ANGLE_DEG);
        assertTrue(Setpoint.MIN_TURN_ANGLE_DEG < -60.0 && Setpoint.MIN_TURN_ANGLE_DEG > -180.0,
            "unexpected min travel: " + Setpoint.MIN_TURN_ANGLE_DEG);
    }
}
