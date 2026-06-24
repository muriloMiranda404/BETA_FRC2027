package frc.robot.subsystems.mechanism.shooter.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.utils.LoggerConstants;

public class TurretSubsystem extends SubsystemBase {

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;

    private final Supplier<Pose2d> poseSupplier;

    private WantedState wantedState = WantedState.OFF;
    private SystemState currentState = SystemState.OFF;
    private SystemState lastState = null;

    private double aimingSetpointDeg = 0.0;
    private Translation3d currentTarget = null;

    public TurretSubsystem(TurretIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {
        io.processInputs(inputs);
        Logger.processInputs(LoggerConstants.MECHANISM_KEY + "Turret/", inputs);

        if (currentTarget != null && wantedState == WantedState.AIMING) {
            aimingSetpointDeg = TurretCalculator.calculateTurretAngle(
                poseSupplier.get(), currentTarget
            );
        }

        currentState = handleTransition();
        applyState();
    }

    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public void setTarget(Translation3d target) {
        this.currentTarget = target;
    }

    public void setAimingSetpoint(double degrees) {
        this.aimingSetpointDeg = degrees;
    }

    public boolean atSetpoint() {
        return inputs.atSetpoint;
    }

    public SystemState getCurrentState() {
        return currentState;
    }

    private SystemState handleTransition() {
        return switch (wantedState) {
            case AIMING -> SystemState.AIMING;
            case SAVED -> SystemState.SAVED;
            case PASSING -> SystemState.PASSING;
            case OFF -> SystemState.OFF;
        };
    }

    private void applyState() {
        switch (currentState) {
            case AIMING -> io.setTurretPosition(aimingSetpointDeg);
            case SAVED -> io.setTurretPosition(0.0);
            case PASSING -> io.setTurretPosition(90.0);
            case OFF -> {
                if (lastState != SystemState.OFF) {
                    io.stop();
                }
            }
        }
        lastState = currentState;
    }

    public enum SystemState {
        AIMING,
        SAVED,
        PASSING,
        OFF
    }

    public enum WantedState {
        AIMING,
        SAVED,
        PASSING,
        OFF
    }
}
