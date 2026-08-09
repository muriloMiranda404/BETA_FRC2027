package frc.frc_java9485.sensor;

import static frc.frc_java9485.constants.utils.LoggerConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalSensor implements SensorIO{

    private final DigitalInput sensor;
    private final boolean inverted;
    private final String identification;

    public DigitalSensor(int input, boolean inverted, String identification){
        this.sensor = new DigitalInput(input);
        this.inverted = inverted;
        this.identification = identification;
    }

    public DigitalSensor(int input, String identification){
        this(input, false, identification);
    }

    @Override
    public void processInput(SensorInputsAutoLogged sensorInputs) {
        sensorInputs.detected = isDetected();
        sensorInputs.inverted = inverted;

        Logger.processInputs(DIGITAL_SENSOR_KEY + identification, sensorInputs);
    }

    @Override
    public boolean isDetected() {
        boolean isDetected = isInverted() ? !sensor.get() : sensor.get();

        return isDetected;
    }

    @Override
    public boolean isInverted() {
        return inverted;
    }
}
