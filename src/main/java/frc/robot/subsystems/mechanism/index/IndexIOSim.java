package frc.robot.subsystems.mechanism.index;

import static edu.wpi.first.units.Units.Volts;

import frc.frc_java9485.constants.SimConsts;
import frc.frc_java9485.sim.RollerSim;


public class IndexIOSim implements IndexIO {

    private final RollerSim sim = new RollerSim(new RollerSim.Config()
            .withMotor(SimConsts.Index.MOTOR)
            .withGearing(SimConsts.Index.GEARING)
            .withMomentOfInertia(SimConsts.Index.MOI_KG_M2));

    private double commandedOutput = 0.0;

    @Override
    public void indexBalls(double speed) {
        this.commandedOutput = speed;
    }

    @Override
    public void stopIndex() {
        this.commandedOutput = 0.0;
    }

    @Override
    public void processInputs(IndexInputsAutoLogged inputs) {
        sim.setVoltageClamped(commandedOutput * 12.0);
        sim.simulate();

        inputs.indexSpeed = sim.getVelocityRPM();
        inputs.isCollecting = sim.getVelocityRPM() > 1.0;
        inputs.current = sim.getCurrentAmps();
        inputs.voltage = Volts.of(commandedOutput * 12.0);
    }
}
