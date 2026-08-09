package frc.robot.subsystems.mechanism.index;

import static edu.wpi.first.units.Units.Volts;

import frc.frc_java9485.constants.robot.CanIds;
import frc.frc_java9485.motors.ctre.TalonFXMotor;
import frc.frc_java9485.motors.ctre.io.TalonFXInputsAutoLogged;
import frc.frc_java9485.utils.logger.VirtualPD;


public class IndexIOTalonFX implements IndexIO {

    private final TalonFXMotor indexer;
    private final TalonFXInputsAutoLogged indexerInputs;

    private double commandedOutput;

    public IndexIOTalonFX() {
        this.indexer = new TalonFXMotor(CanIds.Mechanisms.INDEX, "Index Motor");
        this.indexerInputs = new TalonFXInputsAutoLogged();

        configureIndex();
    }

    private void configureIndex() {
        indexer.setInverted(false);

        indexer.setIdleMode(false);
        indexer.resetPosition(0.0);

        VirtualPD.registerMotor(indexer::getStatorCurrent, "Index", "Index");
    }

    @Override
    public void processInputs(IndexInputsAutoLogged inputs) {
        inputs.indexSpeed = indexer.getRPM();
        inputs.isCollecting = indexer.getRPM() > 1.0;
        inputs.current = indexer.getStatorCurrent();
        inputs.voltage = Volts.of(indexer.getVoltage());

        indexer.updateInputs(indexerInputs);
    }

    @Override
    public void indexBalls(double speed) {
        this.commandedOutput = speed;
        indexer.setSpeed(speed);
    }

    @Override
    public void stopIndex() {
        this.commandedOutput = 0.0;
        indexer.setSpeed(0.0);
    }


    public double getCommandedOutput() {
        return commandedOutput;
    }
}
