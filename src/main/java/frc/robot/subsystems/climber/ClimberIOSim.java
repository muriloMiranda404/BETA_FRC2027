package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import frc.frc_java9485.constants.SimConsts;
import frc.frc_java9485.constants.mechanisms.ClimberConsts;
import frc.frc_java9485.sim.LinearSim;


public class ClimberIOSim implements ClimberIO {

    private static final double LIMIT_WINDOW = 0.5;

    private final LinearSim sim = new LinearSim(new LinearSim.Config()
            .withMotor(SimConsts.Climber.MOTOR)
            .withGearing(SimConsts.Climber.GEARING)
            .withCarriageMass(SimConsts.Climber.CARRIAGE_MASS_KG)
            .withDrumRadius(SimConsts.Climber.DRUM_RADIUS_M)
            .withTravel(0.0, SimConsts.Climber.TRAVEL_M)
            .withGravity(true)
            .withStartingHeight(0.0));

    private double commandedOutput = 0.0;

    @Override
    public void setOutput(double percent) {
        this.commandedOutput = MathUtil.clamp(percent, -1.0, 1.0);
    }

    @Override
    public void stop() {
        this.commandedOutput = 0.0;
    }

    @Override
    public void resetEncoder(double position) {
        sim.setState(toMeters(position), 0.0);
    }

    @Override
    public void processInputs(ClimberIOInputsAutoLogged inputs) {
        sim.setVoltageClamped(commandedOutput * 12.0);
        sim.simulate();

        double position = toEncoderUnits(sim.getPosition());

        inputs.position = position;
        inputs.velocity = toEncoderUnits(sim.getVelocity());
        inputs.current = sim.getCurrentAmps();
        inputs.appliedVolts = commandedOutput * 12.0;
        inputs.atTop = position >= ClimberConsts.MAX_POSITION - LIMIT_WINDOW;
        inputs.atBottom = position <= ClimberConsts.MIN_POSITION + LIMIT_WINDOW;
    }

    private static double toEncoderUnits(double meters) {
        double range = ClimberConsts.MAX_POSITION - ClimberConsts.MIN_POSITION;
        return ClimberConsts.MIN_POSITION + (meters / SimConsts.Climber.TRAVEL_M) * range;
    }

    private static double toMeters(double encoderUnits) {
        double range = ClimberConsts.MAX_POSITION - ClimberConsts.MIN_POSITION;
        return ((encoderUnits - ClimberConsts.MIN_POSITION) / range) * SimConsts.Climber.TRAVEL_M;
    }
}
