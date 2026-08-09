package frc.frc_java9485.motors.ctre.phoenix6;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.junit.jupiter.api.Test;


class TalonFXConfigEqualityTest {

    @Test
    void identicalConfigsAreEqual() {
        assertTrue(TalonFXConfigEquality.isEqual(new TalonFXConfiguration(), new TalonFXConfiguration()));
        assertTrue(TalonFXConfigEquality.isEqual(
                TalonFXFactory.getDefaultConfig(), TalonFXFactory.getDefaultConfig()));
    }

    @Test
    void differentPidGainsAreNotEqual() {
        TalonFXConfiguration a = new TalonFXConfiguration();
        TalonFXConfiguration b = new TalonFXConfiguration();
        b.Slot0.kP = 12.0;

        assertFalse(TalonFXConfigEquality.isEqual(a, b));
    }

    @Test
    void differentNeutralModeIsNotEqual() {
        TalonFXConfiguration a = new TalonFXConfiguration();
        TalonFXConfiguration b = new TalonFXConfiguration();
        a.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        b.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        assertFalse(TalonFXConfigEquality.isEqual(a, b));
    }

    @Test
    void differentInversionIsNotEqual() {
        TalonFXConfiguration a = new TalonFXConfiguration();
        TalonFXConfiguration b = new TalonFXConfiguration();
        a.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        b.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        assertFalse(TalonFXConfigEquality.isEqual(a, b));
    }

    @Test
    void differentCurrentLimitIsNotEqual() {
        TalonFXConfiguration a = new TalonFXConfiguration();
        TalonFXConfiguration b = new TalonFXConfiguration();
        a.CurrentLimits.StatorCurrentLimit = 40.0;
        a.CurrentLimits.StatorCurrentLimitEnable = true;
        b.CurrentLimits.StatorCurrentLimit = 80.0;
        b.CurrentLimits.StatorCurrentLimitEnable = true;

        assertFalse(TalonFXConfigEquality.isEqual(a, b));
    }

    @Test
    void differentGearRatioIsNotEqual() {
        TalonFXConfiguration a = new TalonFXConfiguration();
        TalonFXConfiguration b = new TalonFXConfiguration();
        a.Feedback.SensorToMechanismRatio = 1.0;
        b.Feedback.SensorToMechanismRatio = 66.8;

        assertFalse(TalonFXConfigEquality.isEqual(a, b));
    }


    @Test
    void differencesWithinEpsilonAreTolerated() {
        TalonFXConfiguration a = new TalonFXConfiguration();
        TalonFXConfiguration b = new TalonFXConfiguration();
        a.Slot0.kP = 1.0;
        b.Slot0.kP = 1.0 + TalonFXConfigEquality.TALON_CONFIG_EPSILON / 2.0;

        assertTrue(TalonFXConfigEquality.isEqual(a, b));
    }

    @Test
    void motionMagicChangesAreDetected() {
        TalonFXConfiguration a = new TalonFXConfiguration();
        TalonFXConfiguration b = new TalonFXConfiguration();
        b.MotionMagic.MotionMagicCruiseVelocity = 30.0;

        assertFalse(TalonFXConfigEquality.isEqual(a, b));
    }



    @Test
    void canDeviceId_sameBusAndNumberAreEqual() {
        assertEquals(new CanDeviceId(3, "rio"), new CanDeviceId(3));
        assertEquals(new CanDeviceId(3).hashCode(), new CanDeviceId(3, "rio").hashCode());
    }

    @Test
    void canDeviceId_differentBusIsNotEqual() {
        CanDeviceId onRio = new CanDeviceId(3, "rio");
        CanDeviceId onCanivore = new CanDeviceId(3, "canivore");

        assertNotEquals(onRio, onCanivore);
        assertFalse(onRio.isSameBusAs(onCanivore));
    }

    @Test
    void followerOnAnotherBusIsRejected() {
        CanDeviceId leader = new CanDeviceId(1, "rio");
        CanDeviceId follower = new CanDeviceId(2, "canivore");


        assertThrows(
                IllegalArgumentException.class,
                () -> TalonFXFactory.createPermanentFollowerTalon(follower, leader, false));
    }
}
