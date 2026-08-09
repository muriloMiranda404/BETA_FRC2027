package frc.robot.subsystems.mechanism.shooter.turret;

import frc.frc_java9485.bases.ServoMechanism;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;


public class TurretSubsystem extends ServoMechanism<TurretSubsystem.WantedState, TurretSubsystem.SystemState, TurretIOInputsAutoLogged> {


    private static final double PASSING_ANGLE_DEG = 90.0;

    private final TurretIO io;

    public TurretSubsystem(TurretIO io) {
        super("Turret",
              new TurretIOInputsAutoLogged(),
              WantedState.OFF,
              SystemState.OFF,
              TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG,
              TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG,
              TurretConsts.Setpoint.TOLERANCE_DEG);
        this.io = io;
    }


    public void setAimingSetpoint(double degrees) {
        setSetpoint(degrees);
    }

    @Override
    public double getMeasuredPosition() {
        return inputs.turretAngle;
    }

    @Override
    protected void readInputs(TurretIOInputsAutoLogged inputs) {
        io.processInputs(inputs);
    }

    @Override
    protected SystemState handleTransition(WantedState wanted) {
        return switch (wanted) {
            case AIMING -> SystemState.AIMING;
            case SAVED -> SystemState.SAVED;
            case PASSING -> SystemState.PASSING;
            case OFF -> SystemState.OFF;
        };
    }

    @Override
    protected void applyState(SystemState state, boolean stateChanged) {
        switch (state) {

            case AIMING -> io.setTurretPosition(getSetpoint());
            case SAVED -> {
                setSetpoint(0.0);
                io.setTurretPosition(getSetpoint());
            }
            case PASSING -> {
                setSetpoint(PASSING_ANGLE_DEG);
                io.setTurretPosition(getSetpoint());
            }
            case OFF -> {

                if (stateChanged) {
                    io.stop();
                }
            }
        }
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
