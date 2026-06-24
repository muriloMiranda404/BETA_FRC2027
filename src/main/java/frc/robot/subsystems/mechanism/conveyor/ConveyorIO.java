package frc.robot.subsystems.mechanism.conveyor;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface ConveyorIO {

    @AutoLog
    public static class ConveyorInputs{
       public double speed = 0;
       public boolean atHome = false;
       public boolean atLimit = false;
       public Voltage voltage = Volts.of(0);
       public boolean isLocked = false;
    }

    default void runToMax(){};

    default void runToMin(){};

    default void stop(){};

    boolean atHome();

    boolean atLimit();

    default void processInputs(ConveyorInputsAutoLogged inputs){};
}
