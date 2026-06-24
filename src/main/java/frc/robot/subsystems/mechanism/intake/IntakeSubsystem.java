package frc.robot.subsystems.mechanism.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.mechanisms.IntakeConsts;
import frc.frc_java9485.constants.utils.LoggerConstants;

public class IntakeSubsystem extends SubsystemBase{

  private IntakeIO io;

  private SystemState currentState = SystemState.SAVED;
  private WantedState wantedState = WantedState.SAVED;

  private final IntakeInputsAutoLogged inputs;

  public IntakeSubsystem(IntakeIO io){
    this.io = io;

    this.inputs = new IntakeInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.processInputs(inputs);
    Logger.processInputs(LoggerConstants.MECHANISM_KEY+"Intake/", inputs);

    this.currentState = handleTransition();
    this.executeActions();
  }

  private SystemState handleTransition(){
    return SystemState.valueOf(wantedState.name());
  }

  private void executeActions(){
    switch (currentState) {
      case COLLECTING:
          io.setColectOutput(IntakeConsts.Setpoint.COLLECT_FUEL_SPEED);
          io.setPivotPosition(IntakeConsts.Setpoint.SETPOINT_DOWN);
        break;

      case SAVED:
          io.stopColect();
          io.setPivotPosition(IntakeConsts.Setpoint.SETPOINT_UP);
        break;

      case EJECTING:
          io.setColectOutput(-IntakeConsts.Setpoint.COLLECT_FUEL_SPEED);
          io.setPivotPosition(IntakeConsts.Setpoint.SETPOINT_DOWN);
        break;
      default:
        break;
    }
  }

  public WantedState getWantedState(){
    return wantedState;
  }

  public SystemState getSystemState(){
    return currentState;
  }

  public void setWantedState(WantedState state){
    this.wantedState = state;
  }

  private enum SystemState{
    COLLECTING,
    EJECTING,
    SAVED
  }

  public enum WantedState{
    COLLECTING,
    EJECTING,
    SAVED
  }
}
