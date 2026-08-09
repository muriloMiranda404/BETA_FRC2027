package frc.robot.subsystems.mechanism.shooter.flyWheel;

import frc.frc_java9485.bases.FlywheelMechanism;
import frc.frc_java9485.constants.mechanisms.shooter.FlyWheelConsts;


public class FlyWheelSubsystem extends FlywheelMechanism<FlyWheelSubsystem.WantedState, FlyWheelSubsystem.SystemState, FlyWheelIOInputsAutoLogged> {

    private final FlyWheelIO io;

    private double shootingRPM = 0.0;
    private double passingRPM = 0.0;

    public FlyWheelSubsystem(FlyWheelIO io) {
        super("Fly Wheels",
              new FlyWheelIOInputsAutoLogged(),
              WantedState.OFF,
              SystemState.OFF,
              FlyWheelConsts.Setpoint.TOLERANCE_RPM,
              FlyWheelConsts.Setpoint.SPUN_UP_DEBOUNCE_SEC);
        this.io = io;
    }

    public void setShootingRPM(double rpm) {
        this.shootingRPM = rpm;
    }

    public void setPassingRPM(double rpm) {
        this.passingRPM = rpm;
    }

    @Override
    public double getMeasuredRPM() {
        return inputs.averageSpeed;
    }

    @Override
    protected void readInputs(FlyWheelIOInputsAutoLogged inputs) {
        io.processInputs(inputs);
    }

    @Override
    protected SystemState handleTransition(WantedState wanted) {
        return switch (wanted) {
            case SHOOTING -> SystemState.SHOOTING;
            case PASSING -> SystemState.PASSING;
            case OFF -> SystemState.OFF;
        };
    }

    @Override
    protected void applyState(SystemState state, boolean stateChanged) {
        switch (state) {
            case SHOOTING -> {
                setSetpointRPM(shootingRPM);
                io.setFlyWheelSpeed(shootingRPM);
            }
            case PASSING -> {
                setSetpointRPM(passingRPM);
                io.setFlyWheelSpeed(passingRPM);
            }
            case OFF -> {
                setSetpointRPM(0.0);
                io.stop();
            }
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
