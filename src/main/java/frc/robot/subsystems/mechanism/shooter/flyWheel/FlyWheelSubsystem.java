package frc.robot.subsystems.mechanism.shooter.flyWheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.utils.LoggerConstants;

public class FlyWheelSubsystem extends SubsystemBase {

    private final FlyWheelIO io;
    private final FlyWheelIOInputsAutoLogged inputs;

    private WantedState wantedState = WantedState.OFF;
    private SystemState currentState = SystemState.OFF;

    private double shootingRPM = 0.0;
    private double passingRPM = 0.0;

    public FlyWheelSubsystem(FlyWheelIO io) {
        this.io = io;
        this.inputs = new FlyWheelIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.processInputs(inputs);
        Logger.processInputs(LoggerConstants.MECHANISM_KEY + "Fly Wheels/", inputs);

        currentState = handleTransition();
        executeActions();
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public void setShootingRPM(double rpm) {
        this.shootingRPM = rpm;
    }

    public void setPassingRPM(double rpm) {
        this.passingRPM = rpm;
    }

    private SystemState handleTransition() {
        return switch (wantedState) {
            case SHOOTING -> SystemState.SHOOTING;
            case PASSING -> SystemState.PASSING;
            case OFF -> SystemState.OFF;
        };
    }

    private void executeActions() {
        switch (currentState) {
            case SHOOTING -> io.setFlyWheelSpeed(shootingRPM);
            case PASSING -> io.setFlyWheelSpeed(passingRPM);
            case OFF -> io.stop();
        }
    }

    public enum SystemState {
        SHOOTING,
        PASSING,
        OFF
    }

    public enum WantedState {
        SHOOTING,
        PASSING,
        OFF
    }
}
