package frc.robot.subsystems.mechanism.shooter.hood;

import com.revrobotics.spark.SparkBase.ControlType;

import frc.frc_java9485.constants.mechanisms.shooter.HoodConsts;
import frc.frc_java9485.motors.rev.SparkMaxMotor;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;

public class HoodIOSparkMax implements HoodIO{

    private final SparkMaxMotor hoodMotor;

    private final SparkInputsAutoLogged hoodMotorInputs;

    private double hoodSetpoint;

    public HoodIOSparkMax(){
        this.hoodMotor = new SparkMaxMotor(HoodConsts.Motor.HOOD_MOTOR_ID, "Hood Motor");

        this.hoodMotorInputs = new SparkInputsAutoLogged();

        this.configureHood();
    }

    private void configureHood(){
        this.hoodMotor.setCurrentLimit(HoodConsts.Configs.HOOD_CURRENT_LIMIT);
        this.hoodMotor.setClosedLoopPID(HoodConsts.PID.Kp,
                                        HoodConsts.PID.Ki,
                                        HoodConsts.PID.Kd);
        this.hoodMotor.setInverted(false);
        this.hoodMotor.burnFlash();
        this.hoodMotor.resetPositionByEncoder(0);
    }

    @Override
    public void processInputs(HoodIOInputsAutoLogged inputs) {
        inputs.atHome = hoodMotor.getPosition() == HoodConsts.Setpoint.MIN_POSITION;
        inputs.atLimit = hoodMotor.getPosition() == HoodConsts.Setpoint.MAX_POSITION;
        inputs.atSetpoint = hoodMotor.atSetpoint();
        inputs.hoodSetpoit = hoodSetpoint;
        inputs.hoodAngleDeg = hoodMotor.getPosition();

        this.hoodMotor.updateInputs(hoodMotorInputs);
    }

    @Override
    public void setHoodFromSetpoint(double position) {
        this.hoodSetpoint = position;
        this.hoodMotor.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void resetHood() {
        this.hoodMotor.resetPositionByEncoder(0);
    }

    @Override
    public void returnHoodToHome() {
        this.hoodSetpoint = 0;
        this.hoodMotor.setSetpoint(0, ControlType.kPosition);
    }

    @Override
    public void off() {
        this.hoodSetpoint = hoodMotor.getPosition();
        this.hoodMotor.setSetpoint(hoodMotor.getPosition(), ControlType.kPosition);
    }
}
