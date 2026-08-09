package frc.robot.subsystems.climber;

import frc.frc_java9485.constants.mechanisms.ClimberConsts;
import frc.frc_java9485.constants.robot.CanIds;
import frc.frc_java9485.motors.ctre.TalonFXMotor;
import frc.frc_java9485.motors.ctre.io.TalonFXInputsAutoLogged;
import frc.frc_java9485.utils.logger.VirtualPD;


public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFXMotor motor;
    private final TalonFXInputsAutoLogged motorInputs;

    public ClimberIOTalonFX() {
        this.motor = new TalonFXMotor(CanIds.Mechanisms.CLIMBER, "Climber Motor");
        this.motorInputs = new TalonFXInputsAutoLogged();

        configureClimber();
    }

    private void configureClimber() {
        motor.setCurrentLimit(ClimberConsts.CURRENT_LIMIT);
        motor.setInverted(false);
        motor.setIdleMode(true);

        motor.setForwardSoftLimit(ClimberConsts.MAX_POSITION);
        motor.setReverseSoftLimit(ClimberConsts.MIN_POSITION);
        motor.enableForwardSoftLimit(true);
        motor.enableReverseSoftLimit(true);

        VirtualPD.registerMotor(motor::getStatorCurrent, "Climber", "Climber");
    }

    @Override
    public void setOutput(double percent) {
        motor.setVoltage(percent * 12.0);
    }

    @Override
    public void stop() {
        motor.setVoltage(0.0);
    }

    @Override
    public void resetEncoder(double position) {
        motor.resetPosition(position);
    }

    @Override
    public void processInputs(ClimberIOInputsAutoLogged inputs) {
        double position = motor.getPosition();

        inputs.position = position;
        inputs.velocity = motor.getVelocity();
        inputs.current = motor.getStatorCurrent();
        inputs.appliedVolts = motor.getVoltage();
        inputs.atTop = position >= ClimberConsts.MAX_POSITION;
        inputs.atBottom = position <= ClimberConsts.MIN_POSITION;

        motor.updateInputs(motorInputs);
    }
}
