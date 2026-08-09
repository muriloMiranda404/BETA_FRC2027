package frc.frc_java9485.bases;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.frc_java9485.constants.utils.LoggerConstants;
import frc.frc_java9485.utils.logger.LoggedTracer;


public abstract class StateMachineMechanism<W extends Enum<W>, S extends Enum<S>, In extends LoggableInputs> {


    protected final String name;


    protected final String logKey;

    protected final In inputs;

    private W wantedState;
    private S currentState;
    private S lastState;

    protected StateMachineMechanism(String name, In inputs, W initialWantedState, S initialSystemState) {
        this.name = name;
        this.logKey = LoggerConstants.MECHANISM_KEY + name + "/";
        this.inputs = inputs;
        this.wantedState = initialWantedState;
        this.currentState = initialSystemState;
        this.lastState = null;
    }


    public final void update() {
        readInputs(inputs);
        Logger.processInputs(logKey, inputs);

        beforeTransition();

        S next = handleTransition(wantedState);
        boolean stateChanged = next != currentState;
        lastState = currentState;
        currentState = next;

        applyState(currentState, stateChanged);

        Logger.recordOutput(logKey + "WantedState", wantedState.toString());
        Logger.recordOutput(logKey + "SystemState", currentState.toString());
        recordOutputs();

        LoggedTracer.record(name);
    }




    protected abstract void readInputs(In inputs);


    protected abstract S handleTransition(W wanted);


    protected abstract void applyState(S state, boolean stateChanged);


    protected void beforeTransition() {}


    protected void recordOutputs() {}



    public final void setWantedState(W wantedState) {
        this.wantedState = wantedState;
    }

    public final W getWantedState() {
        return wantedState;
    }

    public final S getCurrentState() {
        return currentState;
    }


    public final S getLastState() {
        return lastState;
    }

    public final String getName() {
        return name;
    }
}
