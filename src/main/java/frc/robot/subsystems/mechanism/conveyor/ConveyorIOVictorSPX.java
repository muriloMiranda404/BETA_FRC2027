package frc.robot.subsystems.mechanism.conveyor;


import frc.frc_java9485.constants.mechanisms.ConveyorConsts;
import frc.frc_java9485.motors.ctre.VictorSPXMotor;
import frc.frc_java9485.motors.ctre.io.CtreMotorInputsAutoLogged;
import frc.frc_java9485.sensor.DigitalSensor;
import frc.frc_java9485.sensor.SensorInputsAutoLogged;

public class ConveyorIOVictorSPX implements ConveyorIO{

    private final VictorSPXMotor conveyor;
    private final CtreMotorInputsAutoLogged conveyorMotorInputs;
    private final ConveyorInputsAutoLogged conveyorInputs;

    private final DigitalSensor limitSensor;
    private final DigitalSensor homeSensor;

    private final SensorInputsAutoLogged limitSensorInputs;
    private final SensorInputsAutoLogged homeSensorInputs;

    public ConveyorIOVictorSPX(){
        this.conveyor = new VictorSPXMotor(ConveyorConsts.Motor.CONVEYOR_MOTOR_ID, "Conveyor Motor");

        this.conveyorMotorInputs = new CtreMotorInputsAutoLogged();
        this.limitSensorInputs = new SensorInputsAutoLogged();
        this.homeSensorInputs = new SensorInputsAutoLogged();
        this.conveyorInputs = new ConveyorInputsAutoLogged();

        this.limitSensor = new DigitalSensor(ConveyorConsts.Sensors.LIMIT_SENSOR_ID, "Limit Sensor");
        this.homeSensor = new DigitalSensor(ConveyorConsts.Sensors.HOME_SENSOR_ID, ConveyorConsts.Sensors.INVERT_HOME, "Home Sensor");
    }

    @Override
    public void stop() {
        this.conveyor.setSpeed(0);
    }

    @Override
    public void runToMax() {
        if(limitSensor.isDetected()) {
            stop();
            return;
        }
        this.conveyor.setSpeed(1);
    }

    @Override
    public void runToMin() {
        if(homeSensor.isDetected()){
            stop();
            return;
        }
        this.conveyor.setSpeed(-1);
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
        inputs.atHome = homeSensor.isDetected();
        inputs.atLimit = limitSensor.isDetected();

        this.conveyor.updateInputs(conveyorMotorInputs);
        this.homeSensor.processInput(homeSensorInputs);
        this.limitSensor.processInput(limitSensorInputs);
    }
}
