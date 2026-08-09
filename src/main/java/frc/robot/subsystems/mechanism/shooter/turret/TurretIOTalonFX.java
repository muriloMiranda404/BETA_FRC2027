package frc.robot.subsystems.mechanism.shooter.turret;

import edu.wpi.first.math.MathUtil;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.frc_java9485.constants.robot.CanIds;
import frc.frc_java9485.motors.ctre.TalonFXMotor;
import frc.frc_java9485.motors.ctre.io.TalonFXInputsAutoLogged;
import frc.frc_java9485.utils.logger.VirtualPD;


public class TurretIOTalonFX implements TurretIO {

    private final TalonFXMotor motor;
    private final TalonFXInputsAutoLogged motorInputs;

    private double setpointDeg;

    public TurretIOTalonFX() {
        this.motor = new TalonFXMotor(CanIds.Mechanisms.TURRET, "Turret Motor");
        this.motorInputs = new TalonFXInputsAutoLogged();

        configureTurret();
    }

    private void configureTurret() {

        motor.setSensorToMechanismRatio(TurretConsts.Config.TURRET_REDUCTION);

        motor.setCurrentLimit(TurretConsts.Config.TURRET_CURRENT_LIMIT);
        motor.setClosedLoopPID(TurretConsts.PID.Kp, TurretConsts.PID.Ki, TurretConsts.PID.Kd);
        motor.setClosedLoopFeedForward(TurretConsts.PID.Ks, 0.0, 0.0);
        motor.setInverted(false);
        motor.setIdleMode(true);


        motor.setForwardSoftLimit(degreesToRotations(TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG));
        motor.setReverseSoftLimit(degreesToRotations(TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG));
        motor.enableForwardSoftLimit(true);
        motor.enableReverseSoftLimit(true);

        motor.resetPosition(0.0);

        VirtualPD.registerMotor(motor::getStatorCurrent, "Turret", "Shooter");
    }

    @Override
    public void processInputs(TurretIOInputsAutoLogged inputs) {
        double angleDeg = getTurretPosition();

        inputs.turretAngle = angleDeg;
        inputs.turretSetpoint = setpointDeg;
        inputs.atSetpoint = atSetpoint();
        inputs.atMax = angleDeg >= TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG - 0.5;
        inputs.atMin = angleDeg <= TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG + 0.5;
        inputs.isBrakeMode = true;

        motor.updateInputs(motorInputs);
    }

    @Override
    public void setTurretPosition(double degrees) {

        double clampedDeg = MathUtil.clamp(
                degrees,
                TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG,
                TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG);

        this.setpointDeg = clampedDeg;
        motor.setPositionSetpoint(degreesToRotations(clampedDeg));
    }

    @Override
    public void lockTurret() {
        motor.setIdleMode(true);
    }

    @Override
    public void reset() {
        motor.resetPosition(0.0);
        this.setpointDeg = 0.0;
    }

    @Override
    public void stop() {

        setTurretPosition(getTurretPosition());
    }

    @Override
    public boolean atSetpoint() {
        return Math.abs(setpointDeg - getTurretPosition()) <= TurretConsts.Setpoint.TOLERANCE_DEG;
    }

    @Override
    public double getTurretPosition() {
        return motor.getPosition() * 360.0;
    }

    private static double degreesToRotations(double degrees) {
        return degrees / 360.0;
    }
}
