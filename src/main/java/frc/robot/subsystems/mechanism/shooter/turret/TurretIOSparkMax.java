package frc.robot.subsystems.mechanism.shooter.turret;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.frc_java9485.motors.rev.SparkMaxMotor;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;

public class TurretIOSparkMax implements TurretIO{

    private final SparkMaxMotor turretMotor;

    private final TurretIOInputsAutoLogged turretInputs;
    private final SparkInputsAutoLogged turretMotorInputs;

    private double turretSetpoint;

    public TurretIOSparkMax(){
        this.turretMotor = new SparkMaxMotor(TurretConsts.Motors.TURRET_MOTOR, "Turret Motor");

        this.turretInputs = new TurretIOInputsAutoLogged();
        this.turretMotorInputs = new SparkInputsAutoLogged();
    }

    @Override
    public void processInputs(TurretIOInputsAutoLogged inputs) {
        inputs.atMax = turretMotor.getPosition() >= TurretConsts.Setpoint.MAX_TURN_POSITION;
        inputs.atMin = turretMotor.getPosition() <= TurretConsts.Setpoint.MIN_TURN_POSITION;
        inputs.atSetpoint = turretMotor.atSetpoint();
        inputs.isBrakeMode = turretMotor.getIdleMode() == IdleMode.kBrake;
        inputs.turretAngle = turretMotor.getPosition();
        inputs.turretSetpoint = turretSetpoint;

        turretMotor.updateInputs(turretMotorInputs);
    }

    @Override
    public void setTurretPosition(double position) {
        this.turretSetpoint = position;
        turretMotor.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void lockTurret() {
        this.turretMotor.setIdleMode(true);
    }

    @Override
    public void reset() {
        this.turretMotor.resetPositionByEncoder(0);
    }

    @Override
    public void stop() {
        turretMotor.setSetpoint(turretMotor.getPosition(), ControlType.kPosition);
    }

    @Override
    public boolean atSetpoint() {
        return turretInputs.atSetpoint;
    }

    @Override
    public double getTurretPosition() {
        return turretInputs.turretAngle;
    }
}
