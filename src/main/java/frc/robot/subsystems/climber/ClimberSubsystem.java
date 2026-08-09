package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsystem extends SubsystemBase {

    private final ClimberMechanism mechanism;

    public ClimberSubsystem(ClimberIO io) {
        this.mechanism = new ClimberMechanism(io);
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
            mechanism.setWantedState(ClimberMechanism.WantedState.OFF);
        }

        mechanism.update();
    }

    public void setWantedState(ClimberMechanism.WantedState wantedState) {
        mechanism.setWantedState(wantedState);
    }

    public ClimberMechanism.WantedState getWantedState() {
        return mechanism.getWantedState();
    }


    public double getPosition() {
        return mechanism.getPosition();
    }

    public ClimberMechanism.SystemState getCurrentState() {
        return mechanism.getCurrentState();
    }


    public Command extend() {
        return startEnd(
            () -> setWantedState(ClimberMechanism.WantedState.EXTENDING),
            () -> setWantedState(ClimberMechanism.WantedState.OFF));
    }


    public Command retract() {
        return startEnd(
            () -> setWantedState(ClimberMechanism.WantedState.RETRACTING),
            () -> setWantedState(ClimberMechanism.WantedState.OFF));
    }
}
