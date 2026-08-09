package frc.frc_java9485.motors.ctre;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.frc_java9485.constants.utils.LoggerConstants.TALON_FX_KEY;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.frc_java9485.motors.ctre.io.TalonFXIO;
import frc.frc_java9485.motors.ctre.io.TalonFXInputsAutoLogged;
import frc.frc_java9485.motors.ctre.phoenix6.CanDeviceId;
import frc.frc_java9485.motors.ctre.phoenix6.Phoenix6Util;
import frc.frc_java9485.motors.ctre.phoenix6.StatusSignalRefresher;
import frc.frc_java9485.motors.ctre.phoenix6.TalonFXFactory;


public class TalonFXMotor implements TalonFXIO {


    private static final int SLOW_SIGNAL_DELAY_LOOPS = 9;

    private final TalonFX motor;
    private final String name;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private GravityTypeValue gravityType;


    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final PositionVoltage positionRequest = new PositionVoltage(0.0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);


    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Double> dutyCycleSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    private double targetPosition = Double.NaN;
    private double targetVelocity = Double.NaN;
    private double targetOutput = Double.NaN;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State trapezoidSetpoint = new TrapezoidProfile.State();


    private Velocity<VoltageUnit> quasistaticVoltagePerSecond;
    private Voltage dynamicVoltage;
    private Time sysIdTimeout;
    private SysIdRoutine sysIdRoutine;

    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutAngle sysIdPosition = Rotations.mutable(0);
    private final MutAngularVelocity sysIdVelocity = RotationsPerSecond.mutable(0);

    public TalonFXMotor(int id, String name) {
        this(new CanDeviceId(id), GravityTypeValue.Elevator_Static, name);
    }

    public TalonFXMotor(CanDeviceId id, String name) {
        this(id, GravityTypeValue.Elevator_Static, name);
    }


    public TalonFXMotor(CanDeviceId id, GravityTypeValue gravityType, String name) {
        this.name = name;
        this.gravityType = gravityType;

        this.motor = TalonFXFactory.createDefaultTalon(id);

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        statorCurrentSignal = motor.getStatorCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();
        voltageSignal = motor.getMotorVoltage();
        dutyCycleSignal = motor.getDutyCycle();
        temperatureSignal = motor.getDeviceTemp();

        StatusSignalRefresher refresher = StatusSignalRefresher.getInstance();
        refresher.addStatusSignals(
                positionSignal, velocitySignal, statorCurrentSignal, voltageSignal, dutyCycleSignal);
        refresher.addStatusSignals(SLOW_SIGNAL_DELAY_LOOPS, supplyCurrentSignal, temperatureSignal);
    }



    @Override
    public void updateInputs(TalonFXInputsAutoLogged inputs) {
        inputs.id = motor.getDeviceID();
        inputs.inverted = config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
        inputs.isFollower = isFollower();
        inputs.isBrakeMode = config.MotorOutput.NeutralMode == NeutralModeValue.Brake;
        inputs.dutyCycle = dutyCycleSignal.getValueAsDouble();
        inputs.currentVoltage = Volts.of(voltageSignal.getValueAsDouble());
        inputs.currentPosition = Rotations.of(positionSignal.getValueAsDouble());
        inputs.statorAmps = Amps.of(statorCurrentSignal.getValueAsDouble());
        inputs.supplyAmps = Amps.of(supplyCurrentSignal.getValueAsDouble());
        inputs.currentRPM = RPM.of(getRPM());
        inputs.currentTemperature = Celsius.of(temperatureSignal.getValueAsDouble());
        inputs.positionSetpoint = Rotations.of(Double.isNaN(targetPosition) ? 0.0 : targetPosition);
        inputs.speedSetpoint = RPM.of(Double.isNaN(targetVelocity) ? 0.0 : targetVelocity * 60.0);
        inputs.faults = motor.getFaultField().getValue();

        Logger.processInputs(TALON_FX_KEY + name, inputs);
    }



    @Override
    public String getMotorName() {
        return name;
    }

    @Override
    public int getDeviceId() {
        return motor.getDeviceID();
    }

    @Override
    public boolean isFollower() {
        return motor.getAppliedControl() instanceof Follower;
    }

    @Override
    public Object getMotor() {
        return motor;
    }



    @Override
    public void setSpeed(double percentOutput) {
        targetOutput = percentOutput;
        targetPosition = Double.NaN;
        targetVelocity = Double.NaN;
        motor.setControl(dutyCycleRequest.withOutput(percentOutput));
    }

    @Override
    public void setVoltage(double volts) {
        targetOutput = volts;
        targetPosition = Double.NaN;
        targetVelocity = Double.NaN;
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        setVoltage(voltage.in(Volts));
    }

    @Override
    public void stop() {
        targetOutput = 0.0;
        targetPosition = Double.NaN;
        targetVelocity = Double.NaN;
        motor.stopMotor();
    }



    @Override
    public void setPositionSetpoint(double position) {
        setPositionSetpoint(position, 0.0);
    }

    @Override
    public void setPositionSetpoint(double position, double feedForward) {
        targetPosition = position;
        targetOutput = Double.NaN;
        targetVelocity = Double.NaN;
        motor.setControl(positionRequest.withPosition(position).withFeedForward(feedForward));
    }

    @Override
    public void setMotionMagicSetpoint(double position) {
        targetPosition = position;
        targetOutput = Double.NaN;
        targetVelocity = Double.NaN;
        motor.setControl(motionMagicRequest.withPosition(position));
    }

    @Override
    public void setVelocitySetpoint(double velocity) {
        setVelocitySetpoint(velocity, 0.0);
    }

    @Override
    public void setVelocitySetpoint(double velocity, double feedForward) {
        targetVelocity = velocity;
        targetOutput = Double.NaN;
        targetPosition = Double.NaN;
        motor.setControl(velocityRequest.withVelocity(velocity).withFeedForward(feedForward));
    }

    @Override
    public void configureTrapezoid(double maxAcceleration, double maxVelocity) {
        this.motionProfile =
                new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        this.trapezoidSetpoint = new TrapezoidProfile.State(getPosition(), getVelocity());
    }

    @Override
    public void setTrapezoidSetpoint(double dtSeconds, double positionGoal, double velocityGoal) {
        setTrapezoidSetpoint(dtSeconds, positionGoal, velocityGoal, 0.0);
    }

    @Override
    public void setTrapezoidSetpoint(
            double dtSeconds, double positionGoal, double velocityGoal, double feedForward) {
        if (motionProfile == null) {
            throw new IllegalStateException(
                    "configureTrapezoid() must be called before setTrapezoidSetpoint() on " + name);
        }
        trapezoidSetpoint = motionProfile.calculate(
                dtSeconds, trapezoidSetpoint, new TrapezoidProfile.State(positionGoal, velocityGoal));
        setPositionSetpoint(trapezoidSetpoint.position, feedForward);
    }



    @Override
    public void factoryDefault() {
        applyConfig(TalonFXFactory.getDefaultConfig());
    }

    @Override
    public void clearStickyFaults() {
        motor.clearStickyFaults();
    }

    @Override
    public void setClosedLoopPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.GravityType = gravityType;
        applyConfig(config);
    }

    @Override
    public void setClosedLoopFeedForward(double kS, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        applyConfig(config);
    }

    @Override
    public void setGravityFeedForward(double kG) {
        config.Slot0.kG = kG;
        config.Slot0.GravityType = gravityType;
        applyConfig(config);
    }

    @Override
    public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = acceleration;
        config.MotionMagic.MotionMagicJerk = jerk;
        applyConfig(config);
    }

    @Override
    public void enableContinuousWrap(boolean enable) {
        config.ClosedLoopGeneral.ContinuousWrap = enable;
        applyConfig(config);
    }

    @Override
    public void setInverted(boolean inverted) {
        config.MotorOutput.Inverted =
                inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        applyConfig(config);
    }

    @Override
    public void setIdleMode(boolean isBrake) {
        config.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        applyConfig(config);
    }

    @Override
    public void setCurrentLimit(int amps) {
        config.CurrentLimits.StatorCurrentLimit = amps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        applyConfig(config);
    }

    @Override
    public void setRampRate(double seconds) {
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = seconds;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = seconds;
        applyConfig(config);
    }

    @Override
    public void setSensorToMechanismRatio(double ratio) {
        config.Feedback.SensorToMechanismRatio = ratio;
        applyConfig(config);
    }

    @Override
    public void setForwardSoftLimit(double limit) {
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = limit;
        applyConfig(config);
    }

    @Override
    public void setReverseSoftLimit(double limit) {
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = limit;
        applyConfig(config);
    }

    @Override
    public void enableForwardSoftLimit(boolean enable) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
        applyConfig(config);
    }

    @Override
    public void enableReverseSoftLimit(boolean enable) {
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
        applyConfig(config);
    }

    @Override
    public void followMotor(int leaderId, boolean opposeLeaderDirection) {
        motor.setControl(new Follower(
                leaderId, opposeLeaderDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
    }

    @Override
    public void resetPosition(double position) {
        Phoenix6Util.checkError(motor.setPosition(position), "Failed to reset position on " + name);
    }


    private void applyConfig(TalonFXConfiguration configToApply) {
        Phoenix6Util.applyAndCheckConfiguration(motor, configToApply);
    }



    @Override
    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    @Override
    public double getRPM() {
        return velocitySignal.getValueAsDouble() * 60.0;
    }

    @Override
    public double getVoltage() {
        return voltageSignal.getValueAsDouble();
    }

    @Override
    public double getStatorCurrent() {
        return statorCurrentSignal.getValueAsDouble();
    }

    @Override
    public double getSupplyCurrent() {
        return supplyCurrentSignal.getValueAsDouble();
    }

    @Override
    public double getTemperature() {
        return temperatureSignal.getValueAsDouble();
    }

    @Override
    public double getDutyCycle() {
        return dutyCycleSignal.getValueAsDouble();
    }

    @Override
    public boolean getLimitSwitch(boolean forward) {
        return forward
                ? motor.getForwardLimit().getValue().value == 0
                : motor.getReverseLimit().getValue().value == 0;
    }

    @Override
    public void reportFaults(String subsystemName) {
        Phoenix6Util.checkFaults(subsystemName, motor);
        Phoenix6Util.checkStickyFaults(subsystemName, motor);
    }


    public double getTargetPosition() {
        return targetPosition;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetOutput() {
        return targetOutput;
    }

    public TalonFX getTalon() {
        return motor;
    }



    @Override
    public void configureSysId(double quasistaticVoltsPerSecond, double dynamicVolts, double timeoutSeconds) {
        this.quasistaticVoltagePerSecond = Volts.of(quasistaticVoltsPerSecond).per(Second);
        this.dynamicVoltage = Volts.of(dynamicVolts);
        this.sysIdTimeout = Seconds.of(timeoutSeconds);
    }

    @Override
    public void buildSysIdRoutine(Subsystem owner) {
        buildSysIdRoutine(owner, null);
    }

    @Override
    public void buildSysIdRoutine(Subsystem owner, TalonFXIO otherMotor) {
        requireSysIdConfigured();
        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(quasistaticVoltagePerSecond, dynamicVoltage, sysIdTimeout),
                new SysIdRoutine.Mechanism(
                        voltage -> {
                            setVoltage(voltage);
                            if (otherMotor != null) {
                                otherMotor.setVoltage(voltage);
                            }
                        },
                        log -> {
                            logSysIdMotor(log, this);
                            if (otherMotor != null) {
                                logSysIdMotor(log, otherMotor);
                            }
                        },
                        owner));
    }

    private void logSysIdMotor(SysIdRoutineLog log, TalonFXIO target) {
        log.motor(target.getMotorName())
                .voltage(appliedVoltage.mut_replace(target.getVoltage(), Volts))
                .angularPosition(sysIdPosition.mut_replace(target.getPosition(), Rotations))
                .angularVelocity(sysIdVelocity.mut_replace(target.getVelocity(), RotationsPerSecond));
    }

    @Override
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        requireSysIdRoutine();
        return sysIdRoutine.quasistatic(direction);
    }

    @Override
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        requireSysIdRoutine();
        return sysIdRoutine.dynamic(direction);
    }

    private void requireSysIdConfigured() {
        if (quasistaticVoltagePerSecond == null || dynamicVoltage == null || sysIdTimeout == null) {
            throw new IllegalStateException("configureSysId() must be called before building the routine on " + name);
        }
    }

    private void requireSysIdRoutine() {
        if (sysIdRoutine == null) {
            throw new IllegalStateException("buildSysIdRoutine() must be called first on " + name);
        }
    }
}
