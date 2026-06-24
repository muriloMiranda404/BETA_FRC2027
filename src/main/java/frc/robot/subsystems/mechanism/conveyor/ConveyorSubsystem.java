package frc.robot.subsystems.mechanism.conveyor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.utils.LoggerConstants;

public class ConveyorSubsystem extends SubsystemBase{

    private final ConveyorIO io;

    private WantedState wantedState = WantedState.STOPPED;
    private SystemState currentState = SystemState.STOPPED;

    private final ConveyorInputsAutoLogged inputs;

    public ConveyorSubsystem(ConveyorIO io){
        this.io = io;

        this.inputs = new ConveyorInputsAutoLogged();
    }

    private SystemState handleTransition(){
        return SystemState.valueOf(wantedState.name());
    }

    public void setWantedState(WantedState state){
        this.wantedState = state;
    }

    private void executeAction(){
        switch (currentState) {
            case STOPPED:
                io.stop();
            break;

            case WITHDRAWING:
                io.runToMin();
            break;

            case EXPANDING:
                io.runToMax();
            break;

            default:
                break;
        }
    }

    public boolean atHome(){
        return io.atHome();
    }

    public boolean atLimit(){
        return io.atLimit();
    }

    @Override
    public void periodic() {
        io.processInputs(inputs);
        Logger.processInputs(LoggerConstants.MECHANISM_KEY+"Conveyor/", inputs);

        this.currentState = handleTransition();
        this.executeAction();
    }

    private enum SystemState{
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
