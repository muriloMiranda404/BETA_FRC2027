package frc.robot.subsystems.mechanism.shooter.hood;

import edu.wpi.first.math.MathUtil;
import frc.frc_java9485.constants.mechanisms.shooter.HoodConsts;
import frc.frc_java9485.constants.robot.CanIds;
import frc.frc_java9485.motors.ctre.TalonFXMotor;
import frc.frc_java9485.motors.ctre.io.TalonFXInputsAutoLogged;
import frc.frc_java9485.utils.logger.VirtualPD;

import com.ctre.phoenix6.signals.GravityTypeValue;


public class HoodIOTalonFX implements HoodIO {

    private static final double POSITION_TOLERANCE = 0.05;


    private static final double HOME_WINDOW = 0.05;

    private final TalonFXMotor motor;
    private final TalonFXInputsAutoLogged motorInputs;

    private double setpoint;

    public HoodIOTalonFX() {
        this.motor = new TalonFXMotor(
                new frc.frc_java9485.motors.ctre.phoenix6.CanDeviceId(CanIds.Mechanisms.HOOD),
                GravityTypeValue.Arm_Cosine,
                "Hood Motor");
        this.motorInputs = new TalonFXInputsAutoLogged();

        configureHood();
    }

    private void configureHood() {
        motor.setCurrentLimit(HoodConsts.Configs.HOOD_CURRENT_LIMIT);
        motor.setClosedLoopPID(HoodConsts.PID.Kp, HoodConsts.PID.Ki, HoodConsts.PID.Kd);
        motor.setClosedLoopFeedForward(HoodConsts.PID.Ks, 0.0, 0.0);
        motor.setGravityFeedForward(HoodConsts.PID.Kg);
        motor.setInverted(false);
        motor.setIdleMode(true);

        motor.setForwardSoftLimit(HoodConsts.Setpoint.MAX_POSITION);
        motor.setReverseSoftLimit(HoodConsts.Setpoint.MIN_POSITION);
        motor.enableForwardSoftLimit(true);
        motor.enableReverseSoftLimit(true);

        motor.resetPosition(HoodConsts.Setpoint.MIN_POSITION);

        VirtualPD.registerMotor(motor::getStatorCurrent, "Hood", "Shooter");
    }

    @Override
    public void processInputs(HoodIOInputsAutoLogged inputs) {
        double position = motor.getPosition();

        inputs.hoodPosition = position;
        inputs.hoodSetpoint = setpoint;
        inputs.atSetpoint = Math.abs(setpoint - position) <= POSITION_TOLERANCE;
        inputs.atHome = position <= HoodConsts.Setpoint.MIN_POSITION + HOME_WINDOW;
        inputs.atLimit = position >= HoodConsts.Setpoint.MAX_POSITION - HOME_WINDOW;

        motor.updateInputs(motorInputs);
    }

    @Override
    public void setHoodFromSetpoint(double position) {
        this.setpoint = MathUtil.clamp(
                position, HoodConsts.Setpoint.MIN_POSITION, HoodConsts.Setpoint.MAX_POSITION);
        motor.setPositionSetpoint(this.setpoint);
    }

    @Override
    public void returnHoodToHome() {
        setHoodFromSetpoint(HoodConsts.Setpoint.MIN_POSITION);
    }

    @Override
    public void off() {

        setHoodFromSetpoint(motor.getPosition());
    }

    @Override
    public void resetHood() {
        motor.resetPosition(HoodConsts.Setpoint.MIN_POSITION);
        this.setpoint = HoodConsts.Setpoint.MIN_POSITION;
    }
}
