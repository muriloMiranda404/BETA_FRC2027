package frc.robot.subsystems.mechanism.conveyor;

import static edu.wpi.first.units.Units.Volts;

import frc.frc_java9485.constants.SimConsts;
import frc.frc_java9485.sim.LinearSim;


public class ConveyorIOSim implements ConveyorIO {


    private static final double SENSOR_WINDOW = 0.02;

    private final LinearSim sim = new LinearSim(new LinearSim.Config()
            .withMotor(SimConsts.Conveyor.MOTOR)
            .withGearing(SimConsts.Conveyor.GEARING)
            .withCarriageMass(SimConsts.Conveyor.CARRIAGE_MASS_KG)
            .withDrumRadius(SimConsts.Conveyor.DRUM_RADIUS_M)
            .withTravel(0.0, SimConsts.Conveyor.TRAVEL_M)
            .withGravity(false)
            .withStartingHeight(0.0));

    private double commandedOutput = 0.0;

    @Override
    public void runToMax() {
        this.commandedOutput = 1.0;
    }

    @Override
    public void runToMin() {
        this.commandedOutput = -1.0;
    }

    @Override
    public void stop() {
        this.commandedOutput = 0.0;
    }

    @Override
    public boolean atHome() {
        return normalizedPosition() <= SENSOR_WINDOW;
    }

    @Override
    public boolean atLimit() {
        return normalizedPosition() >= 1.0 - SENSOR_WINDOW;
    }

    @Override
    public void processInputs(ConveyorInputsAutoLogged inputs) {
        sim.setVoltageClamped(commandedOutput * 12.0);
        sim.simulate();

        inputs.speed = sim.getVelocity();
        inputs.atHome = atHome();
        inputs.atLimit = atLimit();
        inputs.voltage = Volts.of(commandedOutput * 12.0);
        inputs.isLocked = commandedOutput == 0.0;
    }


    private double normalizedPosition() {
        return sim.getPosition() / SimConsts.Conveyor.TRAVEL_M;
    }
}
