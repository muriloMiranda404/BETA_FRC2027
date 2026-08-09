package frc.robot.subsystems.mechanism.index;

import frc.frc_java9485.bases.StateMachineMechanism;
import frc.frc_java9485.constants.mechanisms.IndexConsts;


public class IndexSubsystem extends StateMachineMechanism<IndexSubsystem.WantedState, IndexSubsystem.SystemState, IndexInputsAutoLogged> {

    private final IndexIO io;

    public IndexSubsystem(IndexIO io){
        super("Index", new IndexInputsAutoLogged(), WantedState.STOPPED, SystemState.STOPPED);
        this.io = io;
    }

    @Override
    protected void readInputs(IndexInputsAutoLogged inputs) {
        io.processInputs(inputs);
    }

    @Override
    protected SystemState handleTransition(WantedState wanted){
        return switch (wanted) {
            case STOPPED -> SystemState.STOPPED;
            case EJECTING -> SystemState.EJECTING;
            case INDEXING -> SystemState.INDEXING;
        };
    }

    @Override
    protected void applyState(SystemState state, boolean stateChanged){
        switch (state) {
            case STOPPED -> io.stopIndex();
            case EJECTING -> io.indexBalls(-IndexConsts.MAX_SPEED);
            case INDEXING -> io.indexBalls(IndexConsts.MAX_SPEED);
        }
    }

    public enum SystemState{
        STOPPED,
        EJECTING,
        INDEXING
    }

    public enum WantedState{
        STOPPED,
        EJECTING,
        INDEXING
    }
}
