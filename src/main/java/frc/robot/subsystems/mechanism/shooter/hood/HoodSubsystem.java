package frc.robot.subsystems.mechanism.shooter.hood;

import frc.frc_java9485.bases.ServoMechanism;
import frc.frc_java9485.constants.mechanisms.shooter.HoodConsts;


public class HoodSubsystem extends ServoMechanism<HoodSubsystem.WantedState, HoodSubsystem.SystemState, HoodIOInputsAutoLogged> {

    private static final double SETPOINT_TOLERANCE = 0.05;

    private final HoodIO io;

    public HoodSubsystem(HoodIO io) {
        super("Hood",
              new HoodIOInputsAutoLogged(),
              WantedState.OFF,
              SystemState.OFF,
              HoodConsts.Setpoint.MIN_POSITION,
              HoodConsts.Setpoint.MAX_POSITION,
              SETPOINT_TOLERANCE);
        this.io = io;
    }


    public void setHoodPosition(double position) {
        setSetpoint(position);
    }

    @Override
    public double getMeasuredPosition() {
        return inputs.hoodPosition;
    }

    @Override
    protected void readInputs(HoodIOInputsAutoLogged inputs) {
        io.processInputs(inputs);
    }


    @Override
    protected boolean atHomeSensor() {
        return inputs.atHome;
    }

    @Override
    protected void onReachedHome() {
        io.resetHood();
    }

    @Override
    protected SystemState handleTransition(WantedState wanted) {
        return switch (wanted) {
            case ANGLING -> SystemState.ANGLING;
            case HOME -> SystemState.HOME;
            case OFF -> SystemState.OFF;
        };
    }

    @Override
    protected void applyState(SystemState state, boolean stateChanged) {
        switch (state) {
            case ANGLING -> io.setHoodFromSetpoint(getSetpoint());
            case HOME -> {
                setSetpoint(HoodConsts.Setpoint.MIN_POSITION);
                io.returnHoodToHome();
            }
            case OFF -> {

                if (stateChanged) {
                    io.off();
                }
            }
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
