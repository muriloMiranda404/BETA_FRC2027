package frc.robot.commands.mechanism.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.robot.commands.mechanism.shooter.TurretChassisAllocator.Allocation;


class TurretChassisAllocatorTest {

    private static final double MAX = TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG;
    private static final double MIN = TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG;

    @Test
    void turretAloneHandlesAnAngleWellInsideItsTravel() {
        Allocation allocation = TurretChassisAllocator.allocate(30.0, 0.0, false);

        assertEquals(30.0, allocation.turretRelativeDeg(), 1e-9);
        assertFalse(allocation.chassisEngaged(), "no chassis rotation needed for a comfortable angle");
        assertFalse(allocation.turretSaturated());

        assertEquals(0.0, allocation.chassisGoalHeadingDeg(), 1e-9);
    }

    @Test
    void chassisEngagesBeforeTheTurretHitsItsStop() {

        double nearLimit = MAX - TurretChassisAllocator.ENGAGE_MARGIN_DEG / 2.0;
        Allocation allocation = TurretChassisAllocator.allocate(nearLimit, 0.0, false);

        assertTrue(allocation.chassisEngaged(), "chassis should help before the turret runs out");
        assertFalse(allocation.turretSaturated(), "and it should engage while there is still travel left");
    }

    @Test
    void chassisAimsAtTheShotSoTheTurretRecentres() {
        Allocation allocation = TurretChassisAllocator.allocate(140.0, 0.0, false);

        assertTrue(allocation.chassisEngaged());
        assertEquals(140.0, allocation.chassisGoalHeadingDeg(), 1e-9);
        assertTrue(allocation.turretSaturated(), "140 deg is beyond the turret's travel");
        assertEquals(MAX, allocation.turretRelativeDeg(), 1e-9);
    }


    @Test
    void negativeSideIsHandledWithItsOwnLimit() {
        Allocation allocation = TurretChassisAllocator.allocate(-140.0, 0.0, false);

        assertTrue(allocation.chassisEngaged());
        assertEquals(MIN, allocation.turretRelativeDeg(), 1e-9);
        assertTrue(allocation.turretSaturated());
    }


    @Test
    void hysteresisKeepsTheChassisEngagedUntilWellClearOfTheLimit() {

        double betweenMargins = MAX - (TurretChassisAllocator.ENGAGE_MARGIN_DEG
                + TurretChassisAllocator.RELEASE_MARGIN_DEG) / 2.0;

        assertFalse(TurretChassisAllocator.allocate(betweenMargins, 0.0, false).chassisEngaged(),
                "should not engage from rest at this angle");
        assertTrue(TurretChassisAllocator.allocate(betweenMargins, 0.0, true).chassisEngaged(),
                "should stay engaged at the same angle — this is what stops the oscillation");
    }

    @Test
    void chassisReleasesOnceTheTurretIsComfortableAgain() {
        Allocation allocation = TurretChassisAllocator.allocate(0.0, 0.0, true);

        assertFalse(allocation.chassisEngaged(), "centred turret needs no chassis help");
    }


    @Test
    void allocationIsRelativeToTheChassisHeading() {
        Allocation allocation = TurretChassisAllocator.allocate(90.0, 60.0, false);

        assertEquals(30.0, allocation.turretRelativeDeg(), 1e-9);
        assertFalse(allocation.chassisEngaged());
    }


    @Test
    void wrapsAcrossPlusMinus180() {
        Allocation allocation = TurretChassisAllocator.allocate(-170.0, 170.0, false);

        assertEquals(20.0, allocation.turretRelativeDeg(), 1e-9);
        assertFalse(allocation.chassisEngaged());
    }

    @Test
    void remainingTravelIsZeroAtTheLimits() {
        assertEquals(0.0, TurretChassisAllocator.remainingTravelDeg(MAX), 1e-9);
        assertEquals(0.0, TurretChassisAllocator.remainingTravelDeg(MIN), 1e-9);
        assertTrue(TurretChassisAllocator.remainingTravelDeg(0.0) > 50.0);
    }
}
