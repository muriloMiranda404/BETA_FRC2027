package frc.frc_java9485.bases;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.filter.Debouncer;
import frc.frc_java9485.utils.calc.Util;


public abstract class FlywheelMechanism<W extends Enum<W>, S extends Enum<S>, In extends LoggableInputs>
        extends StateMachineMechanism<W, S, In> {

    private final double toleranceRPM;
    private final Debouncer spunUpDebouncer;

    private double setpointRPM = 0.0;
    private boolean spunUpDebounced = false;


    protected FlywheelMechanism(
            String name,
            In inputs,
            W initialWantedState,
            S initialSystemState,
            double toleranceRPM,
            double debounceSeconds) {
        super(name, inputs, initialWantedState, initialSystemState);
        this.toleranceRPM = toleranceRPM;
        this.spunUpDebouncer = new Debouncer(debounceSeconds, Debouncer.DebounceType.kRising);
    }

    public final void setSetpointRPM(double rpm) {
        this.setpointRPM = rpm;
    }

    public final double getSetpointRPM() {
        return setpointRPM;
    }

    public final double getToleranceRPM() {
        return toleranceRPM;
    }


    public abstract double getMeasuredRPM();


    public boolean spunUp() {
        return setpointRPM > 0.0 && Util.epsilonEquals(getMeasuredRPM(), setpointRPM, toleranceRPM);
    }


    public boolean atSetpoint() {
        return spunUpDebounced;
    }

    @Override
    protected void beforeTransition() {
        spunUpDebounced = spunUpDebouncer.calculate(spunUp());
    }

    @Override
    protected void recordOutputs() {
        Logger.recordOutput(logKey + "SetpointRPM", setpointRPM);
        Logger.recordOutput(logKey + "MeasuredRPM", getMeasuredRPM());
        Logger.recordOutput(logKey + "SpunUp", spunUp());
        Logger.recordOutput(logKey + "AtSetpoint", spunUpDebounced);
    }
}
