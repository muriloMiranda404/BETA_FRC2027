package frc.robot.subsystems.mechanism.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import frc.frc_java9485.constants.SimConsts;
import frc.frc_java9485.constants.mechanisms.IntakeConsts;
import frc.frc_java9485.sim.PivotSim;
import frc.frc_java9485.sim.RollerSim;


public class IntakeIOSim implements IntakeIO {

    private final PivotSim pivotSim = new PivotSim(new PivotSim.Config()
            .withMotor(SimConsts.Intake.PIVOT_MOTOR)
            .withGearing(SimConsts.Intake.PIVOT_GEARING)
            .withMomentOfInertia(SimConsts.Intake.PIVOT_MOI_KG_M2)
            .withArmLength(SimConsts.Intake.PIVOT_ARM_LENGTH_M)
            .withHardStops(toRadians(IntakeConsts.Setpoint.SETPOINT_DOWN), toRadians(IntakeConsts.Setpoint.SETPOINT_UP))
            .withGravity(true)
            .withStartingAngle(toRadians(IntakeConsts.Setpoint.SETPOINT_UP)));

    private final RollerSim rollerSim = new RollerSim(new RollerSim.Config()
            .withMotor(SimConsts.Intake.ROLLER_MOTOR)
            .withGearing(SimConsts.Intake.ROLLER_GEARING)
            .withMomentOfInertia(SimConsts.Intake.ROLLER_MOI_KG_M2));

    private final PIDController pivotController =
            new PIDController(SimConsts.Intake.SIM_KP, 0.0, SimConsts.Intake.SIM_KD);

    private double pivotSetpoint = IntakeConsts.Setpoint.SETPOINT_UP;
    private double collectOutput = 0.0;

    @Override
    public void setColectOutput(double porcentage) {
        this.collectOutput = porcentage;
    }

    @Override
    public void setColectVoltage(edu.wpi.first.units.measure.Voltage voltage) {
        this.collectOutput = voltage.in(Volts) / 12.0;
    }

    @Override
    public void setPivotPosition(double position) {
        this.pivotSetpoint = position;
    }

    @Override
    public void stopColect() {
        this.collectOutput = 0.0;
    }

    @Override
    public void processInputs(IntakeInputsAutoLogged inputs) {
        double pivotVolts = pivotController.calculate(measuredPivotPosition(), pivotSetpoint);
        pivotSim.setVoltageClamped(pivotVolts);
        pivotSim.simulate();

        rollerSim.setVoltageClamped(collectOutput * 12.0);
        rollerSim.simulate();

        inputs.pivotAngle = measuredPivotPosition();
        inputs.pivotSetpoint = pivotSetpoint;
        inputs.pivotVolts = Volts.of(pivotVolts);
        inputs.catchFuelSpeed = rollerSim.getVelocityRPM();
        inputs.isColecting = Math.abs(rollerSim.getVelocityRPM()) > 1.0;
        inputs.porcentageColectSetpoint = collectOutput;
        inputs.voltageColectSetpoint = collectOutput * 12.0;
    }


    private double measuredPivotPosition() {
        return pivotSim.getPosition() / SimConsts.Intake.RADIANS_PER_POSITION;
    }

    private static double toRadians(double position) {
        return position * SimConsts.Intake.RADIANS_PER_POSITION;
    }
}
