package frc.frc_java9485.bases;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.MathUtil;
import frc.frc_java9485.utils.calc.Util;


public abstract class ServoMechanism<W extends Enum<W>, S extends Enum<S>, In extends LoggableInputs>
        extends StateMachineMechanism<W, S, In> {

    private final double minPosition;
    private final double maxPosition;
    private final double tolerance;

    private double setpoint;
    private boolean wasAtHome = false;


    protected ServoMechanism(
            String name,
            In inputs,
            W initialWantedState,
            S initialSystemState,
            double minPosition,
            double maxPosition,
            double tolerance) {
        super(name, inputs, initialWantedState, initialSystemState);
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.tolerance = tolerance;
        this.setpoint = MathUtil.clamp(0.0, minPosition, maxPosition);
    }


    public final void setSetpoint(double position) {
        this.setpoint = MathUtil.clamp(position, minPosition, maxPosition);
    }

    public final double getSetpoint() {
        return setpoint;
    }

    public final double getMinPosition() {
        return minPosition;
    }

    public final double getMaxPosition() {
        return maxPosition;
    }

    public final double getTolerance() {
        return tolerance;
    }


    public abstract double getMeasuredPosition();


    public boolean atSetpoint() {
        return Util.epsilonEquals(getMeasuredPosition(), setpoint, tolerance);
    }


    public final boolean nearPosition(double position, double epsilon) {
        return Util.epsilonEquals(getMeasuredPosition(), position, epsilon);
    }




    protected boolean atHomeSensor() {
        return false;
    }


    protected void onReachedHome() {}

    @Override
    protected void beforeTransition() {

        boolean atHome = atHomeSensor();
        if (atHome && !wasAtHome) {
            onReachedHome();
        }
        wasAtHome = atHome;
    }

    @Override
    protected void recordOutputs() {
        Logger.recordOutput(logKey + "Setpoint", setpoint);
        Logger.recordOutput(logKey + "Measured", getMeasuredPosition());
        Logger.recordOutput(logKey + "AtSetpoint", atSetpoint());
    }
}
