package frc.robot.subsystems.mechanism.shooter.flyWheel;

import edu.wpi.first.math.controller.PIDController;
import frc.frc_java9485.constants.SimConsts;
import frc.frc_java9485.constants.mechanisms.shooter.FlyWheelConsts;
import frc.frc_java9485.sim.RollerSim;


public class FlyWheelIOSim implements FlyWheelIO {

    private final RollerSim sim = new RollerSim(new RollerSim.Config()
            .withMotor(SimConsts.FlyWheel.MOTOR)
            .withGearing(SimConsts.FlyWheel.GEARING)
            .withMomentOfInertia(SimConsts.FlyWheel.MOI_KG_M2));

    private final PIDController controller = new PIDController(SimConsts.FlyWheel.SIM_KP, 0.0, 0.0);

    private double setpointRPM = 0.0;

    @Override
    public void setFlyWheelSpeed(double speed) {
        this.setpointRPM = speed;
    }

    @Override
    public void stop() {
        this.setpointRPM = 0.0;
    }

    @Override
    public void processInputs(FlyWheelIOInputsAutoLogged inputs) {
        double measuredRPM = sim.getVelocityRPM();


        double feedforward = setpointRPM * SimConsts.FlyWheel.SIM_KV;
        double feedback = controller.calculate(measuredRPM, setpointRPM);

        sim.setVoltageClamped(setpointRPM <= 0.0 ? 0.0 : Math.max(0.0, feedforward + feedback));
        sim.simulate();

        measuredRPM = sim.getVelocityRPM();

        inputs.averageSpeed = measuredRPM;
        inputs.speedSetpoint = setpointRPM;
        inputs.atSetpoint = setpointRPM > 0.0
                && Math.abs(setpointRPM - measuredRPM) <= FlyWheelConsts.Setpoint.TOLERANCE_RPM;
        inputs.isShooting = measuredRPM > 1.0;
    }
}
