package frc.robot.subsystems.mechanism.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.frc_java9485.constants.SimConsts;
import frc.frc_java9485.constants.mechanisms.shooter.HoodConsts;
import frc.frc_java9485.sim.PivotSim;


public class HoodIOSim implements HoodIO {

    private static final double POSITION_TOLERANCE = 0.05;


    private static final double HOME_SENSOR_WINDOW = 0.05;

    private final PivotSim sim = new PivotSim(new PivotSim.Config()
            .withMotor(SimConsts.Hood.MOTOR)
            .withGearing(SimConsts.Hood.GEARING)
            .withMomentOfInertia(SimConsts.Hood.MOI_KG_M2)
            .withArmLength(SimConsts.Hood.ARM_LENGTH_M)
            .withHardStops(SimConsts.Hood.MIN_ANGLE_RAD, SimConsts.Hood.MAX_ANGLE_RAD)
            .withGravity(true)
            .withStartingAngle(SimConsts.Hood.MIN_ANGLE_RAD));

    private final PIDController controller =
            new PIDController(SimConsts.Hood.SIM_KP, 0.0, SimConsts.Hood.SIM_KD);

    private double setpoint = HoodConsts.Setpoint.MIN_POSITION;

    @Override
    public void setHoodFromSetpoint(double position) {
        this.setpoint = MathUtil.clamp(
                position, HoodConsts.Setpoint.MIN_POSITION, HoodConsts.Setpoint.MAX_POSITION);
    }

    @Override
    public void returnHoodToHome() {
        this.setpoint = HoodConsts.Setpoint.MIN_POSITION;
    }

    @Override
    public void off() {

        this.setpoint = measuredPosition();
    }

    @Override
    public void resetHood() {
        sim.setState(SimConsts.Hood.MIN_ANGLE_RAD, 0.0);
        this.setpoint = HoodConsts.Setpoint.MIN_POSITION;
        controller.reset();
    }

    @Override
    public void processInputs(HoodIOInputsAutoLogged inputs) {
        sim.setVoltageClamped(controller.calculate(measuredPosition(), setpoint));
        sim.simulate();

        double position = measuredPosition();

        inputs.hoodPosition = position;
        inputs.hoodSetpoint = setpoint;
        inputs.atSetpoint = Math.abs(setpoint - position) <= POSITION_TOLERANCE;
        inputs.atHome = position <= HoodConsts.Setpoint.MIN_POSITION + HOME_SENSOR_WINDOW;
        inputs.atLimit = position >= HoodConsts.Setpoint.MAX_POSITION - HOME_SENSOR_WINDOW;
    }


    private double measuredPosition() {
        return sim.getPosition() / SimConsts.Hood.RADIANS_PER_POSITION;
    }
}
