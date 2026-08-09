package frc.robot.subsystems.mechanism.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.frc_java9485.constants.SimConsts;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.frc_java9485.sim.PivotSim;


public class TurretIOSim implements TurretIO {

    private final PivotSim sim = new PivotSim(new PivotSim.Config()
            .withMotor(SimConsts.Turret.MOTOR)
            .withGearing(SimConsts.Turret.GEARING)
            .withMomentOfInertia(SimConsts.Turret.MOI_KG_M2)
            .withArmLength(SimConsts.Turret.ARM_LENGTH_M)
            .withHardStops(SimConsts.Turret.MIN_ANGLE_RAD, SimConsts.Turret.MAX_ANGLE_RAD)

            .withGravity(false)
            .withStartingAngle(0.0));

    private final PIDController controller =
            new PIDController(SimConsts.Turret.SIM_KP, 0.0, SimConsts.Turret.SIM_KD);

    private double setpointDeg = 0.0;
    private boolean closedLoop = true;

    @Override
    public void setTurretPosition(double degrees) {
        this.setpointDeg = MathUtil.clamp(
                degrees,
                TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG,
                TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG);
        this.closedLoop = true;
    }

    @Override
    public void stop() {

        this.setpointDeg = getTurretPosition();
        this.closedLoop = true;
    }

    @Override
    public void reset() {
        sim.setState(0.0, 0.0);
        this.setpointDeg = 0.0;
        controller.reset();
    }

    @Override
    public void lockTurret() {
        this.closedLoop = false;
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(setpointDeg - getTurretPosition()) <= TurretConsts.Setpoint.TOLERANCE_DEG;
    }

    @Override
    public double getTurretPosition() {
        return Math.toDegrees(sim.getPosition());
    }

    @Override
    public void processInputs(TurretIOInputsAutoLogged inputs) {
        double measuredDeg = getTurretPosition();

        sim.setVoltageClamped(closedLoop ? controller.calculate(measuredDeg, setpointDeg) : 0.0);
        sim.simulate();

        measuredDeg = getTurretPosition();

        inputs.turretAngle = measuredDeg;
        inputs.turretSetpoint = setpointDeg;
        inputs.atSetpoint = atSetpoint();
        inputs.atMax = measuredDeg >= TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG - 0.1;
        inputs.atMin = measuredDeg <= TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG + 0.1;
        inputs.isBrakeMode = !closedLoop;
    }
}
