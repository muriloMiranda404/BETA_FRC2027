package frc.robot.subsystems.mechanism.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
  @AutoLog
  public class IntakeInputs{
    public double catchFuelSpeed = 0;
    public boolean isColecting = false;
    public double pivotAngle = 0;
    public double pivotSetpoint = 0;
    public double porcentageColectSetpoint = 0;
    public double voltageColectSetpoint = 0;
    public Voltage pivotVolts = Volts.of(0);
  }

  default void setColectVoltage(Voltage voltage){};

  default void setColectOutput(double porcentage){};

  default void setPivotPosition(double position){};

  default void stopColect(){};

  default void processInputs(IntakeInputsAutoLogged inputs){};
}
