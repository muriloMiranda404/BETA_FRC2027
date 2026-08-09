package frc.frc_java9485.motors.ctre.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public interface TalonFXIO {

    @AutoLog
    public static class TalonFXInputs {
        public int id = 0;
        public boolean inverted = false;
        public boolean isFollower = false;
        public boolean isBrakeMode = false;
        public double dutyCycle = 0.0;
        public Voltage currentVoltage = Volts.of(0);
        public Angle currentPosition = Rotations.of(0);
        public Current statorAmps = Amps.of(0);
        public Current supplyAmps = Amps.of(0);
        public AngularVelocity currentRPM = RPM.of(0);
        public Temperature currentTemperature = Celsius.of(0);
        public Angle positionSetpoint = Rotations.of(0);
        public AngularVelocity speedSetpoint = RPM.of(0);
        public int faults = 0;
    }

    void updateInputs(TalonFXInputsAutoLogged inputs);



    String getMotorName();

    int getDeviceId();

    boolean isFollower();

    Object getMotor();



    void setSpeed(double percentOutput);

    void setVoltage(double volts);

    void setVoltage(Voltage voltage);

    void stop();



    void setPositionSetpoint(double position);

    void setPositionSetpoint(double position, double feedForward);

    void setMotionMagicSetpoint(double position);

    void setVelocitySetpoint(double velocity);

    void setVelocitySetpoint(double velocity, double feedForward);


    void configureTrapezoid(double maxAcceleration, double maxVelocity);

    void setTrapezoidSetpoint(double dtSeconds, double positionGoal, double velocityGoal);

    void setTrapezoidSetpoint(double dtSeconds, double positionGoal, double velocityGoal, double feedForward);



    void factoryDefault();

    void clearStickyFaults();

    void setClosedLoopPID(double kP, double kI, double kD);

    void setClosedLoopFeedForward(double kS, double kV, double kA);

    void setGravityFeedForward(double kG);

    void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk);

    void enableContinuousWrap(boolean enable);

    void setInverted(boolean inverted);

    void setIdleMode(boolean isBrake);

    void setCurrentLimit(int amps);

    void setRampRate(double seconds);

    void setSensorToMechanismRatio(double ratio);

    void setForwardSoftLimit(double limit);

    void setReverseSoftLimit(double limit);

    void enableForwardSoftLimit(boolean enable);

    void enableReverseSoftLimit(boolean enable);

    void followMotor(int leaderId, boolean opposeLeaderDirection);

    void resetPosition(double position);



    double getPosition();

    double getVelocity();

    double getRPM();

    double getVoltage();

    double getStatorCurrent();

    double getSupplyCurrent();

    double getTemperature();

    double getDutyCycle();

    boolean getLimitSwitch(boolean forward);


    void reportFaults(String subsystemName);



    void configureSysId(double quasistaticVoltsPerSecond, double dynamicVolts, double timeoutSeconds);

    void buildSysIdRoutine(Subsystem owner);

    void buildSysIdRoutine(Subsystem owner, TalonFXIO otherMotor);

    Command sysIdQuasistatic(SysIdRoutine.Direction direction);

    Command sysIdDynamic(SysIdRoutine.Direction direction);
}
