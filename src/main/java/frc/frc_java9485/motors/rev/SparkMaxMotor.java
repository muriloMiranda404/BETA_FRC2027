package frc.frc_java9485.motors.rev;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.frc_java9485.constants.utils.LoggerConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.frc_java9485.motors.rev.io.SparkIO;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;

public class SparkMaxMotor implements SparkIO{
  private final SparkMax motor;
  private final SparkMaxConfig config;
  private final String name;
  private final boolean usingAlternateEncoder;

  private double speed = 0;
  private double porcentage = 0;
  private boolean inverted = false;
  private IdleMode lastIdleMode = null;

  private double targetPercentage;
  private double targetPosition;
  private double taregtSpeed;

  public SparkMaxMotor(int id, String name){
    this(id, false, name);
  }

  public SparkMaxMotor(int id, Boolean usingAlternateEncoder, String name) {
    this.motor = new SparkMax(id, MotorType.kBrushless);
    this.config = new SparkMaxConfig();
    this.usingAlternateEncoder = usingAlternateEncoder;
    this.name = name;

    cleanStickFaults();
    this.setAlternateEndoder(usingAlternateEncoder);
  }

  private void setAlternateEndoder(boolean usingAlternateEncoder){
    if(usingAlternateEncoder){
      this.config.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
      this.config.alternateEncoder.countsPerRevolution(8192);
    }

    this.config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  @Override
  public void updateInputs(SparkInputsAutoLogged inputs) {
    inputs.speed = this.speed;
    inputs.id = motor.getDeviceId();
    inputs.inverted = this.inverted;
    inputs.currentRPM = RPM.of(getRPM());
    inputs.currentAmps = Amps.of(getCurrent());
    inputs.currentVoltage = Volts.of(getVoltage());
    inputs.currentPosition = Rotations.of(getPosition());
    inputs.currentTemperature = Celsius.of(getTemperature());
    inputs.positionSetpoint = Rotations.of(getClosedLoopController().getMAXMotionSetpointPosition());
    inputs.speedSetpoint = RPM.of(getClosedLoopController().getMAXMotionSetpointVelocity());

    Logger.processInputs(SPARK_MAX_BRUSHLESS_KEY + name, inputs);
  }

  @Override
  public double getRate() {
      if(usingAlternateEncoder){
        return motor.getAlternateEncoder().getVelocity();
      }
      return motor.getEncoder().getVelocity();
  }

  @Override
  public void setSpeed(double speeds) {
    if (speeds != this.targetPercentage) {
      this.motor
          .getClosedLoopController()
          .setSetpoint(speeds, ControlType.kVelocity);
    }
    this.targetPercentage = speeds;
    this.targetPosition = Double.NaN;
    this.taregtSpeed = Double.NaN;
  }

  @Override
  public double getPosition() {
    return getEncoder().getPosition();
  }

  @Override
  public double getRPM() {
    return getEncoder().getVelocity();
  }

  @Override
  public void setSetpoint(double setpoint, ControlType ctrl) {
    if (setpoint != getPosition()) {
      motor.getClosedLoopController().setSetpoint(setpoint, ctrl);
    }
  }

  @Override
  public void setRampRate(double ramp) {
    config.closedLoopRampRate(ramp).openLoopRampRate(ramp);
  }

  @Override
  public String getMotorName() {
    return name;
  }

  @Override
  public boolean atSetpoint() {
      return getClosedLoopController().isAtSetpoint();
  }

  @Override
  public int getDeviceId() {
    return motor.getDeviceId();
  }

  @Override
  public boolean isFollower() {
    return motor.isFollower();
  }

  @Override
  public boolean isUsingAlternateEncoder() {
    return usingAlternateEncoder;
  }

  @Override
  public double getVoltage() {
    return motor.getBusVoltage();
  }

  public SparkMax getSpark() {
    return motor;
  }

  @Override
  public RelativeEncoder getEncoder() {
    return motor.getEncoder();
  }

  @Override
  public void followMotor(int id) {
    config.follow(id);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    if (voltage.in(Volts) != this.targetPercentage) {
      this.motor.set(voltage.in(Volts));
    }
    this.targetPercentage = voltage.in(Volts);
    this.targetPosition = Double.NaN;
    this.taregtSpeed = Double.NaN;
  }

  @Override
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  @Override
  public void setIdleMode(boolean isBrake) {
    IdleMode targetIdleMode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;
    if (lastIdleMode == targetIdleMode) {
      return;
    }
    config.idleMode(targetIdleMode);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    this.lastIdleMode = targetIdleMode;
  }

  @Override
  public double getCurrent(){
    return motor.getOutputCurrent();
  }

  @Override
  public void setInverted(boolean invert) {
      config.inverted(true);
      this.inverted = invert;
  }

  @Override
  public IdleMode getIdleMode() {
      return lastIdleMode;
  }

  @Override
  public void resetPositionByEncoder(double posisition) {
      getEncoder().setPosition(posisition);
  }

  private void configureSparkMax(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
      Timer.delay(Milliseconds.of(5).in(Seconds));
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  @Override
  public void setCurrentLimit(int current) {
      config.smartCurrentLimit(current);
  }

  @Override
  public void cleanStickFaults() {
      configureSparkMax(motor::clearFaults);
  }

  @Override
  public void setForwardSoftLimit(double limit) {
    config.softLimit.forwardSoftLimit(limit);
  }

  @Override
  public void setReverseSoftLimit(double limit) {
    config.softLimit.reverseSoftLimit(limit);
  }

  @Override
  public void enableForwardSoftLimit(boolean enable) {
    config.softLimit.forwardSoftLimitEnabled(enable);
  }

  @Override
  public void enableReverseSoftLimit(boolean enable) {
    config.softLimit.reverseSoftLimitEnabled(enable);
  }

  @Override
  public void setPositionConversionFactor(double factor) {
    config.encoder.positionConversionFactor(factor);
  }

  @Override
  public void setVelocityConversionFactor(double factor) {
    config.encoder.velocityConversionFactor(factor);
  }

  @Override
  public void setClosedLoopPID(double kP, double kI, double kD) {
    config.closedLoop.pid(kP, kI, kD);
  }

  @Override
  public void setClosedLoopFeedForward(double kA, double kV) {
    config.closedLoop.feedForward.kA(kA);
    config.closedLoop.feedForward.kV(kV);
  }

  @Override
  public void setClosedLoopPhysical(double kS, double kG) {
    config.closedLoop.feedForward.kS(kS);
    config.closedLoop.feedForward.kG(kG);
  }

  @Override
  public void resetConfigToDefault(Motor motor) {
    switch (motor) {
      case NEO_1:
        config.apply(SparkMaxConfig.Presets.REV_NEO);
        break;
      case NEO_2:
        config.apply(SparkMaxConfig.Presets.REV_NEO_2);
        break;
      case NEO_550:
        config.apply(SparkMaxConfig.Presets.REV_NEO_550);
        break;
      case NEO_VORTEX:
        throw new RuntimeException("Cannot configure a Spark Max to NEO Vortex default config");
    }
  }

  @Override
  public SparkClosedLoopController getClosedLoopController() {
      return motor.getClosedLoopController();
  }

  @Override
  public void burnFlash() {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException("Config updates cannot be applied while the robot is Enabled!");
    }
    configureSparkMax(() -> {
      return motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    });
  }
}
