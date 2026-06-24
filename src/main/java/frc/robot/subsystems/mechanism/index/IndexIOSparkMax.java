package frc.robot.subsystems.mechanism.index;

import static edu.wpi.first.units.Units.Volts;


import frc.frc_java9485.constants.mechanisms.IndexConsts;
import frc.frc_java9485.motors.rev.SparkMaxMotor;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;

public class IndexIOSparkMax implements IndexIO{

    private final SparkMaxMotor indexer;

    private final SparkInputsAutoLogged indexMotorInputs;
    private final IndexInputsAutoLogged indexInputs;

    public IndexIOSparkMax(){
        this.indexer = new SparkMaxMotor(IndexConsts.INDEX_ID, "Index Motor");

        this.indexInputs = new IndexInputsAutoLogged();
        this.indexMotorInputs = new SparkInputsAutoLogged();

        this.configureIndexMotor();
    }

    private void configureIndexMotor(){
        this.indexer.resetPositionByEncoder(0);
        this.indexer.setInverted(false);
        this.indexer.burnFlash();
    }

    @Override
    public void processInputs(IndexInputsAutoLogged inputs) {
        inputs.current = indexer.getCurrent();
        inputs.indexSpeed = indexer.getRate();
        inputs.isCollecting = inputs.indexSpeed > 0;
        inputs.voltage = Volts.of(indexer.getVoltage());

        this.indexer.updateInputs(indexMotorInputs);
    }

    @Override
    public void indexBalls(double speed) {
        this.indexer.setSpeed(speed);
    }

    @Override
    public void stopIndex() {
        this.indexer.setSpeed(0);
    }
}
