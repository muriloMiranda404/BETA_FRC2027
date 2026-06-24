package frc.robot.subsystems.mechanism.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.utils.LoggerConstants;

public class HoodSubsystem extends SubsystemBase {

    private static final double SETPOINT_TOLERANCE_DEG = 0.5;

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs;

    private WantedState wantedState = WantedState.OFF;
    private SystemState currentState = SystemState.OFF;

    private double hoodAngleDeg = 0.0;

    public HoodSubsystem(HoodIO io) {
        this.io = io;
        this.inputs = new HoodIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.processInputs(inputs);
        Logger.processInputs(LoggerConstants.MECHANISM_KEY + "Hood/", inputs);

        currentState = handleTransition();
        executeActions();
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public void setHoodAngle(double angleDeg) {
        this.hoodAngleDeg = Math.max(0.0, Math.min(3.5, angleDeg));
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.hoodAngleDeg - hoodAngleDeg) < SETPOINT_TOLERANCE_DEG;
    }

    public SystemState getCurrentState() {
        return currentState;
    }

    private SystemState handleTransition() {
        return switch (wantedState) {
            case ANGLING -> SystemState.ANGLING;
            case HOME -> SystemState.HOME;
            case OFF -> SystemState.OFF;
        };
    }

    private void executeActions() {
        switch (currentState) {
            case ANGLING -> io.setHoodFromSetpoint(hoodAngleDeg);
            case HOME -> io.returnHoodToHome();
            case OFF -> io.off();
        }
    }

    public enum SystemState {
        ANGLING,
        HOME,
        OFF
    }

    public enum WantedState {
        ANGLING,
        HOME,
        OFF
    }
}
