package frc.robot.subsystems.mechanism.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.frc_java9485.constants.mechanisms.IntakeConsts;
import frc.frc_java9485.constants.robot.CanIds;
import frc.frc_java9485.motors.ctre.TalonFXMotor;
import frc.frc_java9485.motors.ctre.io.TalonFXInputsAutoLogged;
import frc.frc_java9485.motors.ctre.phoenix6.CanDeviceId;
import frc.frc_java9485.utils.logger.VirtualPD;


public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFXMotor pivotMotor;
    private final TalonFXMotor collectMotor;

    private final TalonFXInputsAutoLogged pivotInputs;
    private final TalonFXInputsAutoLogged collectInputs;

    private double pivotSetpoint;
    private double collectOutput;

    public IntakeIOTalonFX() {
        this.pivotMotor = new TalonFXMotor(
                new CanDeviceId(CanIds.Mechanisms.INTAKE_PIVOT),
                GravityTypeValue.Arm_Cosine,
                "Intake Pivot Motor");
        this.collectMotor = new TalonFXMotor(CanIds.Mechanisms.INTAKE_ROLLERS, "Intake Collect Motor");

        this.pivotInputs = new TalonFXInputsAutoLogged();
        this.collectInputs = new TalonFXInputsAutoLogged();

        configureIntake();
    }

    private void configureIntake() {
        pivotMotor.setClosedLoopPID(IntakeConsts.PID.Kp, IntakeConsts.PID.Ki, IntakeConsts.PID.Kd);
        pivotMotor.setClosedLoopFeedForward(IntakeConsts.FeedForward.Ks, 0.0, 0.0);
        pivotMotor.setGravityFeedForward(IntakeConsts.FeedForward.Kg);
        pivotMotor.setInverted(false);

        pivotMotor.setIdleMode(true);

        collectMotor.setInverted(false);
        collectMotor.setIdleMode(false);

        VirtualPD.registerMotor(pivotMotor::getStatorCurrent, "Intake Pivot", "Intake");
        VirtualPD.registerMotor(collectMotor::getStatorCurrent, "Intake Rollers", "Intake");
    }

    @Override
    public void processInputs(IntakeInputsAutoLogged inputs) {
        inputs.pivotAngle = pivotMotor.getPosition();
        inputs.pivotSetpoint = pivotSetpoint;
        inputs.pivotVolts = Volts.of(pivotMotor.getVoltage());
        inputs.catchFuelSpeed = collectMotor.getRPM();
        inputs.isColecting = Math.abs(collectMotor.getRPM()) > 1.0;
        inputs.porcentageColectSetpoint = collectOutput;
        inputs.voltageColectSetpoint = collectOutput * 12.0;

        pivotMotor.updateInputs(pivotInputs);
        collectMotor.updateInputs(collectInputs);
    }

    @Override
    public void setColectOutput(double porcentage) {
        this.collectOutput = porcentage;
        collectMotor.setSpeed(porcentage);
    }

    @Override
    public void setColectVoltage(Voltage voltage) {
        this.collectOutput = voltage.in(Volts) / 12.0;
        collectMotor.setVoltage(voltage);
    }

    @Override
    public void setPivotPosition(double position) {
        this.pivotSetpoint = position;
        pivotMotor.setPositionSetpoint(position);
    }

    @Override
    public void stopColect() {
        this.collectOutput = 0.0;
        collectMotor.setSpeed(0.0);
    }
}
