package frc.robot.subsystems.mechanism.conveyor;

import frc.frc_java9485.bases.StateMachineMechanism;


public class ConveyorSubsystem extends StateMachineMechanism<ConveyorSubsystem.WantedState, ConveyorSubsystem.SystemState, ConveyorInputsAutoLogged> {

    private final ConveyorIO io;

    public ConveyorSubsystem(ConveyorIO io){
        super("Conveyor", new ConveyorInputsAutoLogged(), WantedState.STOPPED, SystemState.STOPPED);
        this.io = io;
    }

    @Override
    protected void readInputs(ConveyorInputsAutoLogged inputs) {
        io.processInputs(inputs);
    }

    @Override
    protected SystemState handleTransition(WantedState wanted){
        return switch (wanted) {
            case EXPANDING -> SystemState.EXPANDING;
            case WITHDRAWING -> SystemState.WITHDRAWING;
            case STOPPED -> SystemState.STOPPED;
        };
    }

    @Override
    protected void applyState(SystemState state, boolean stateChanged){
        switch (state) {
            case STOPPED -> io.stop();
            case WITHDRAWING -> io.runToMin();
            case EXPANDING -> io.runToMax();
        }
    }

    public boolean atHome(){
        return io.atHome();
    }

    public boolean atLimit(){
        return io.atLimit();
    }

    public enum SystemState{
        EXPANDING,
        WITHDRAWING,
        STOPPED
    }

    public enum WantedState{
        EXPANDING,
        WITHDRAWING,
        STOPPED
    }
}
