package frc.robot.subsystems.mechanism.intake;

import static edu.wpi.first.units.Units.Volts;


import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Voltage;
import frc.frc_java9485.constants.mechanisms.IntakeConsts;
import frc.frc_java9485.motors.rev.SparkMaxMotor;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;

public class IntakeIOSparkMax implements IntakeIO{

    private final SparkMaxMotor pivotMotor;
    private final SparkMaxMotor colectMotor;

    private final SparkInputsAutoLogged pivotInputs;
    private final SparkInputsAutoLogged colectInputs;
    private final IntakeInputsAutoLogged intakeInputs;

    private double pivotSetpoint;
    private double porcentageColectSetpoint;
    private double voltageColectPorcentage;

    public IntakeIOSparkMax(){
        this.pivotMotor = new SparkMaxMotor(IntakeConsts.Motors.PIVOT_ID, true,"Intake Pivot Motor");
        this.colectMotor = new SparkMaxMotor(IntakeConsts.Motors.CATCH_BALL_ID, "Intake Colect Motor");

        this.colectInputs = new SparkInputsAutoLogged();
        this.pivotInputs = new SparkInputsAutoLogged();
        this.intakeInputs = new IntakeInputsAutoLogged();

        this.configureIntakeComponents();
    }

    private void configureIntakeComponents(){
        this.pivotMotor.setClosedLoopPID(IntakeConsts.PID.Kp,
                                         IntakeConsts.PID.Ki,
                                         IntakeConsts.PID.Kd);
        this.pivotMotor.setClosedLoopPhysical(0, IntakeConsts.FeedForward.Kg);
        this.pivotMotor.setClosedLoopFeedForward(0, IntakeConsts.FeedForward.Kv);
        this.pivotMotor.setInverted(false);
        this.pivotMotor.burnFlash();
    }

    @Override
    public void setColectVoltage(Voltage voltage) {
        this.voltageColectPorcentage = voltage.in(Volts);
        this.colectMotor.setVoltage(voltage);
    }

    @Override
    public void setColectOutput(double porcentage) {
        this.porcentageColectSetpoint = porcentage;
        this.colectMotor.setSpeed(porcentage);
    }

    @Override
    public void setPivotPosition(double position) {
        this.pivotSetpoint = position;
        this.pivotMotor.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void stopColect() {
        this.porcentageColectSetpoint = 0;
        this.colectMotor.setSpeed(0);
    }

    @Override
    public void processInputs(IntakeInputsAutoLogged inputs) {
        inputs.catchFuelSpeed = this.colectMotor.getRate();
        inputs.isColecting = this.colectMotor.getRate() != 0;
        inputs.pivotAngle = this.pivotMotor.getPosition();
        inputs.pivotSetpoint = pivotSetpoint;
        inputs.porcentageColectSetpoint = porcentageColectSetpoint;
        inputs.voltageColectSetpoint = voltageColectPorcentage;
        inputs.pivotVolts = Volts.of(pivotMotor.getVoltage());

        this.colectMotor.updateInputs(colectInputs);
        this.pivotMotor.updateInputs(pivotInputs);
    }
}
