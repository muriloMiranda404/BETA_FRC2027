package frc.frc_java9485.motors.ctre.phoenix6;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;


public class TalonFXConfigEquality {


    public static final boolean ENABLE_LOGGING_INEQ = true;

    public static final double TALON_CONFIG_EPSILON = 0.05;

    private TalonFXConfigEquality() {}

    public static boolean isEqual(TalonFXConfiguration a, TalonFXConfiguration b) {
        return isEqual(a.Slot0, b.Slot0)
                && isEqual(a.Slot1, b.Slot1)
                && isEqual(a.Slot2, b.Slot2)
                && isEqual(a.MotorOutput, b.MotorOutput)
                && isEqual(a.CurrentLimits, b.CurrentLimits)
                && isEqual(a.Voltage, b.Voltage)
                && isEqual(a.TorqueCurrent, b.TorqueCurrent)
                && isEqual(a.Feedback, b.Feedback)
                && isEqual(a.OpenLoopRamps, b.OpenLoopRamps)
                && isEqual(a.ClosedLoopRamps, b.ClosedLoopRamps)
                && isEqual(a.HardwareLimitSwitch, b.HardwareLimitSwitch)
                && isEqual(a.Audio, b.Audio)
                && isEqual(a.SoftwareLimitSwitch, b.SoftwareLimitSwitch)
                && isEqual(a.MotionMagic, b.MotionMagic);
    }

    public static boolean isEqual(Slot0Configs a, Slot0Configs b) {
        boolean val = near(a.kP, b.kP) && near(a.kI, b.kI) && near(a.kD, b.kD) && near(a.kV, b.kV) && near(a.kS, b.kS);
        report(val, "Slot0Configs");
        return val;
    }

    public static boolean isEqual(Slot1Configs a, Slot1Configs b) {
        boolean val = near(a.kP, b.kP) && near(a.kI, b.kI) && near(a.kD, b.kD) && near(a.kV, b.kV) && near(a.kS, b.kS);
        report(val, "Slot1Configs");
        return val;
    }

    public static boolean isEqual(Slot2Configs a, Slot2Configs b) {
        boolean val = near(a.kP, b.kP) && near(a.kI, b.kI) && near(a.kD, b.kD) && near(a.kV, b.kV) && near(a.kS, b.kS);
        report(val, "Slot2Configs");
        return val;
    }

    public static boolean isEqual(MotorOutputConfigs a, MotorOutputConfigs b) {
        boolean val = a.Inverted.value == b.Inverted.value
                && a.NeutralMode.value == b.NeutralMode.value
                && near(a.DutyCycleNeutralDeadband, b.DutyCycleNeutralDeadband)
                && near(a.PeakForwardDutyCycle, b.PeakForwardDutyCycle)
                && near(a.PeakReverseDutyCycle, b.PeakReverseDutyCycle);
        report(val, "MotorOutputConfigs");
        return val;
    }

    public static boolean isEqual(CurrentLimitsConfigs a, CurrentLimitsConfigs b) {
        boolean val = near(a.StatorCurrentLimit, b.StatorCurrentLimit)
                && near(a.SupplyCurrentLimit, b.SupplyCurrentLimit)
                && a.StatorCurrentLimitEnable == b.StatorCurrentLimitEnable
                && a.SupplyCurrentLimitEnable == b.SupplyCurrentLimitEnable;
        report(val, "CurrentLimitsConfigs");
        return val;
    }

    public static boolean isEqual(VoltageConfigs a, VoltageConfigs b) {
        boolean val = near(a.SupplyVoltageTimeConstant, b.SupplyVoltageTimeConstant)
                && near(a.PeakForwardVoltage, b.PeakForwardVoltage)
                && near(a.PeakReverseVoltage, b.PeakReverseVoltage);
        report(val, "VoltageConfigs");
        return val;
    }

    public static boolean isEqual(TorqueCurrentConfigs a, TorqueCurrentConfigs b) {
        boolean val = near(a.PeakForwardTorqueCurrent, b.PeakForwardTorqueCurrent)
                && near(a.PeakReverseTorqueCurrent, b.PeakReverseTorqueCurrent)
                && near(a.TorqueNeutralDeadband, b.TorqueNeutralDeadband);
        report(val, "TorqueCurrentConfigs");
        return val;
    }

    public static boolean isEqual(FeedbackConfigs a, FeedbackConfigs b) {
        boolean val = near(a.FeedbackRotorOffset, b.FeedbackRotorOffset)
                && near(a.SensorToMechanismRatio, b.SensorToMechanismRatio)
                && near(a.RotorToSensorRatio, b.RotorToSensorRatio)
                && a.FeedbackSensorSource.value == b.FeedbackSensorSource.value
                && a.FeedbackRemoteSensorID == b.FeedbackRemoteSensorID;
        if (ENABLE_LOGGING_INEQ && !val) {
            DriverStation.reportWarning(
                    String.format(
                            "FeedbackConfigs not equal: rotorOffset %.4f/%.4f, sensorToMech %.4f/%.4f, "
                                    + "rotorToSensor %.4f/%.4f, source %d/%d, remoteId %d/%d",
                            a.FeedbackRotorOffset, b.FeedbackRotorOffset,
                            a.SensorToMechanismRatio, b.SensorToMechanismRatio,
                            a.RotorToSensorRatio, b.RotorToSensorRatio,
                            a.FeedbackSensorSource.value, b.FeedbackSensorSource.value,
                            a.FeedbackRemoteSensorID, b.FeedbackRemoteSensorID),
                    false);
        }
        return val;
    }

    public static boolean isEqual(OpenLoopRampsConfigs a, OpenLoopRampsConfigs b) {
        boolean val = near(a.DutyCycleOpenLoopRampPeriod, b.DutyCycleOpenLoopRampPeriod)
                && near(a.VoltageOpenLoopRampPeriod, b.VoltageOpenLoopRampPeriod)
                && near(a.TorqueOpenLoopRampPeriod, b.TorqueOpenLoopRampPeriod);
        report(val, "OpenLoopRampsConfigs");
        return val;
    }

    public static boolean isEqual(ClosedLoopRampsConfigs a, ClosedLoopRampsConfigs b) {
        boolean val = near(a.DutyCycleClosedLoopRampPeriod, b.DutyCycleClosedLoopRampPeriod)
                && near(a.VoltageClosedLoopRampPeriod, b.VoltageClosedLoopRampPeriod)
                && near(a.TorqueClosedLoopRampPeriod, b.TorqueClosedLoopRampPeriod);
        report(val, "ClosedLoopRampsConfigs");
        return val;
    }

    public static boolean isEqual(HardwareLimitSwitchConfigs a, HardwareLimitSwitchConfigs b) {
        boolean val = a.ForwardLimitAutosetPositionEnable == b.ForwardLimitAutosetPositionEnable
                && a.ForwardLimitEnable == b.ForwardLimitEnable
                && a.ReverseLimitAutosetPositionEnable == b.ReverseLimitAutosetPositionEnable
                && a.ReverseLimitEnable == b.ReverseLimitEnable
                && near(a.ForwardLimitAutosetPositionValue, b.ForwardLimitAutosetPositionValue)
                && near(a.ReverseLimitAutosetPositionValue, b.ReverseLimitAutosetPositionValue)
                && a.ForwardLimitRemoteSensorID == b.ForwardLimitRemoteSensorID
                && a.ReverseLimitRemoteSensorID == b.ReverseLimitRemoteSensorID
                && a.ForwardLimitSource.value == b.ForwardLimitSource.value
                && a.ReverseLimitSource.value == b.ReverseLimitSource.value
                && a.ForwardLimitType.value == b.ForwardLimitType.value
                && a.ReverseLimitType.value == b.ReverseLimitType.value;
        report(val, "HardwareLimitSwitchConfigs");
        return val;
    }

    public static boolean isEqual(AudioConfigs a, AudioConfigs b) {
        boolean val = a.BeepOnBoot == b.BeepOnBoot;
        report(val, "AudioConfigs");
        return val;
    }

    public static boolean isEqual(SoftwareLimitSwitchConfigs a, SoftwareLimitSwitchConfigs b) {
        boolean val = near(a.ForwardSoftLimitThreshold, b.ForwardSoftLimitThreshold)
                && near(a.ReverseSoftLimitThreshold, b.ReverseSoftLimitThreshold)
                && a.ReverseSoftLimitEnable == b.ReverseSoftLimitEnable
                && a.ForwardSoftLimitEnable == b.ForwardSoftLimitEnable;
        report(val, "SoftwareLimitSwitchConfigs");
        return val;
    }

    public static boolean isEqual(MotionMagicConfigs a, MotionMagicConfigs b) {
        boolean val = near(a.MotionMagicAcceleration, b.MotionMagicAcceleration)
                && near(a.MotionMagicCruiseVelocity, b.MotionMagicCruiseVelocity)
                && near(a.MotionMagicJerk, b.MotionMagicJerk);
        report(val, "MotionMagicConfigs");
        return val;
    }

    private static boolean near(double a, double b) {
        return MathUtil.isNear(a, b, TALON_CONFIG_EPSILON);
    }

    private static void report(boolean equal, String configName) {
        if (ENABLE_LOGGING_INEQ && !equal) {
            DriverStation.reportWarning(configName + " not equal", false);
        }
    }
}
