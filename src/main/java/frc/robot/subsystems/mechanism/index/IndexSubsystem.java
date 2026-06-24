package frc.robot.subsystems.mechanism.index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.mechanisms.IndexConsts;
import frc.frc_java9485.constants.utils.LoggerConstants;

public class IndexSubsystem extends SubsystemBase{

    private IndexIO io;

    private SystemState currenState = SystemState.STOPPED;
    private WantedState wantedState = WantedState.STOPPED;

    private final IndexInputsAutoLogged inputs;

    public IndexSubsystem(IndexIO io){
        this.io = io;

        this.inputs = new IndexInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.processInputs(inputs);
        Logger.processInputs(LoggerConstants.MECHANISM_KEY+"Index/", inputs);

        this.currenState = handleTransition();
        executeActions();
    }

    private SystemState handleTransition(){
        return SystemState.valueOf(wantedState.name());
    }

    private void executeActions(){
        switch (currenState) {
            case STOPPED:
                io.stopIndex();
                break;

            case EJECTING:
                io.indexBalls(-IndexConsts.MAX_SPEED);
                break;
            case INDEXING:
                io.indexBalls(IndexConsts.MAX_SPEED);
                break;
            default:
                break;
        }
    }

    public void setWantedState(WantedState state){
        this.wantedState = state;
    }

    private enum SystemState{
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
