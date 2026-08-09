package frc.robot.subsystems.mechanism.conveyor;

import static edu.wpi.first.units.Units.Volts;

import frc.frc_java9485.constants.mechanisms.ConveyorConsts;
import frc.frc_java9485.constants.robot.CanIds;
import frc.frc_java9485.motors.ctre.TalonFXMotor;
import frc.frc_java9485.motors.ctre.io.TalonFXInputsAutoLogged;
import frc.frc_java9485.sensor.DigitalSensor;
import frc.frc_java9485.sensor.SensorInputsAutoLogged;
import frc.frc_java9485.utils.logger.VirtualPD;


public class ConveyorIOTalonFX implements ConveyorIO {

    private final TalonFXMotor conveyor;
    private final TalonFXInputsAutoLogged conveyorInputs;

    private final DigitalSensor limitSensor;
    private final DigitalSensor homeSensor;

    private final SensorInputsAutoLogged limitSensorInputs;
    private final SensorInputsAutoLogged homeSensorInputs;

    private double commandedOutput;

    public ConveyorIOTalonFX() {
        this.conveyor = new TalonFXMotor(CanIds.Mechanisms.CONVEYOR, "Conveyor Motor");
        this.conveyorInputs = new TalonFXInputsAutoLogged();

        this.limitSensor = new DigitalSensor(ConveyorConsts.Sensors.LIMIT_SENSOR_ID, "Limit Sensor");
        this.homeSensor = new DigitalSensor(
                ConveyorConsts.Sensors.HOME_SENSOR_ID, ConveyorConsts.Sensors.INVERT_HOME, "Home Sensor");

        this.limitSensorInputs = new SensorInputsAutoLogged();
        this.homeSensorInputs = new SensorInputsAutoLogged();

        configureConveyor();
    }

    private void configureConveyor() {
        conveyor.setInverted(false);

        conveyor.setIdleMode(true);

        VirtualPD.registerMotor(conveyor::getStatorCurrent, "Conveyor", "Conveyor");
    }

    @Override
    public void stop() {
        this.commandedOutput = 0.0;
        conveyor.setSpeed(0.0);
    }

    @Override
    public void runToMax() {
        if (limitSensor.isDetected()) {
            stop();
            return;
        }
        this.commandedOutput = ConveyorConsts.Motor.MAX_SPEED;
        conveyor.setSpeed(commandedOutput);
    }

    @Override
    public void runToMin() {
        if (homeSensor.isDetected()) {
            stop();
            return;
        }
        this.commandedOutput = -ConveyorConsts.Motor.MAX_SPEED;
        conveyor.setSpeed(commandedOutput);
    }

    @Override
    public boolean atHome() {
        return homeSensor.isDetected();
    }

    @Override
    public boolean atLimit() {
        return limitSensor.isDetected();
    }

    @Override
    public void processInputs(ConveyorInputsAutoLogged inputs) {
        inputs.speed = conveyor.getRPM();
        inputs.atHome = atHome();
        inputs.atLimit = atLimit();
        inputs.voltage = Volts.of(conveyor.getVoltage());
        inputs.isLocked = commandedOutput == 0.0;

        conveyor.updateInputs(conveyorInputs);
        homeSensor.processInput(homeSensorInputs);
        limitSensor.processInput(limitSensorInputs);
    }
}
