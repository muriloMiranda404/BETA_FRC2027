package frc.robot.subsystems.mechanism.intake;

import frc.frc_java9485.bases.StateMachineMechanism;
import frc.frc_java9485.constants.mechanisms.IntakeConsts;


public class IntakeSubsystem extends StateMachineMechanism<IntakeSubsystem.WantedState, IntakeSubsystem.SystemState, IntakeInputsAutoLogged> {

  private final IntakeIO io;

  public IntakeSubsystem(IntakeIO io){
    super("Intake", new IntakeInputsAutoLogged(), WantedState.SAVED, SystemState.SAVED);
    this.io = io;
  }

  @Override
  protected void readInputs(IntakeInputsAutoLogged inputs) {
    io.processInputs(inputs);
  }

  @Override
  protected SystemState handleTransition(WantedState wanted){
    return switch (wanted) {
      case COLLECTING -> SystemState.COLLECTING;
      case EJECTING -> SystemState.EJECTING;
      case SAVED -> SystemState.SAVED;
    };
  }

  @Override
  protected void applyState(SystemState state, boolean stateChanged){
    switch (state) {
      case COLLECTING -> {
        io.setColectOutput(IntakeConsts.Setpoint.COLLECT_FUEL_SPEED);
        io.setPivotPosition(IntakeConsts.Setpoint.SETPOINT_DOWN);
      }
      case EJECTING -> {
        io.setColectOutput(-IntakeConsts.Setpoint.COLLECT_FUEL_SPEED);
        io.setPivotPosition(IntakeConsts.Setpoint.SETPOINT_DOWN);
      }
      case SAVED -> {
        io.stopColect();
        io.setPivotPosition(IntakeConsts.Setpoint.SETPOINT_UP);
      }
    }
  }


  public double getPivotPosition(){
    return inputs.pivotAngle;
  }

  public enum SystemState{
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
