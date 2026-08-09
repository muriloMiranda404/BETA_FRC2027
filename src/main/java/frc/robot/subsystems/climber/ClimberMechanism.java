package frc.robot.subsystems.climber;

import frc.frc_java9485.bases.StateMachineMechanism;
import frc.frc_java9485.constants.mechanisms.ClimberConsts;


public class ClimberMechanism extends StateMachineMechanism<ClimberMechanism.WantedState, ClimberMechanism.SystemState, ClimberIOInputsAutoLogged> {

    private final ClimberIO io;

    public ClimberMechanism(ClimberIO io) {
        super("Climber", new ClimberIOInputsAutoLogged(), WantedState.OFF, SystemState.OFF);
        this.io = io;
    }

    @Override
    protected void readInputs(ClimberIOInputsAutoLogged inputs) {
        io.processInputs(inputs);
    }

    @Override
    protected SystemState handleTransition(WantedState wanted) {
        return switch (wanted) {
            case EXTENDING -> SystemState.EXTENDING;
            case RETRACTING -> SystemState.RETRACTING;
            case OFF -> SystemState.OFF;
        };
    }

    @Override
    protected void applyState(SystemState state, boolean stateChanged) {
        switch (state) {

            case EXTENDING -> io.setOutput(inputs.atTop ? 0.0 : ClimberConsts.EXTEND_OUTPUT);
            case RETRACTING -> io.setOutput(inputs.atBottom ? 0.0 : ClimberConsts.RETRACT_OUTPUT);
            case OFF -> io.stop();
        }
    }


    public double getPosition() {
        return inputs.position;
    }

    public enum WantedState {
        EXTENDING,
        RETRACTING,
        OFF
    }

    public enum SystemState {
        EXTENDING,
        RETRACTING,
        OFF
    }
}
