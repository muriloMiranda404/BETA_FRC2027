package frc.robot.subsystems.mechanism.shooter.flyWheel;

import frc.frc_java9485.constants.mechanisms.shooter.FlyWheelConsts;
import frc.frc_java9485.constants.robot.CanIds;
import frc.frc_java9485.motors.ctre.TalonFXMotor;
import frc.frc_java9485.motors.ctre.io.TalonFXInputsAutoLogged;
import frc.frc_java9485.utils.logger.VirtualPD;


public class FlyWheelIOTalonFX implements FlyWheelIO {

    private static final double SECONDS_PER_MINUTE = 60.0;

    private final TalonFXMotor rightMotor;
    private final TalonFXMotor leftMotor;
    private final TalonFXMotor indexer;

    private final TalonFXInputsAutoLogged rightInputs;
    private final TalonFXInputsAutoLogged leftInputs;
    private final TalonFXInputsAutoLogged indexerInputs;

    private double setpointRPM;

    public FlyWheelIOTalonFX() {
        this.rightMotor = new TalonFXMotor(CanIds.Mechanisms.FLYWHEEL_RIGHT, "Right Shooter");
        this.leftMotor = new TalonFXMotor(CanIds.Mechanisms.FLYWHEEL_LEFT, "Left Shooter");
        this.indexer = new TalonFXMotor(CanIds.Mechanisms.TURRET_INDEXER, "Turret Indexer");

        this.rightInputs = new TalonFXInputsAutoLogged();
        this.leftInputs = new TalonFXInputsAutoLogged();
        this.indexerInputs = new TalonFXInputsAutoLogged();

        configureFlyWheel();
    }

    private void configureFlyWheel() {
        rightMotor.setCurrentLimit(FlyWheelConsts.Config.SHOOTER_CURRENT_LIMIT);
        rightMotor.setClosedLoopPID(FlyWheelConsts.PID.Kp, FlyWheelConsts.PID.Ki, FlyWheelConsts.PID.Kd);
        rightMotor.setClosedLoopFeedForward(FlyWheelConsts.PID.Ks, FlyWheelConsts.PID.Kv, 0.0);
        rightMotor.setInverted(false);

        rightMotor.setIdleMode(false);

        leftMotor.setCurrentLimit(FlyWheelConsts.Config.SHOOTER_CURRENT_LIMIT);
        leftMotor.setIdleMode(false);
        leftMotor.followMotor(CanIds.Mechanisms.FLYWHEEL_RIGHT, true);

        indexer.setCurrentLimit(FlyWheelConsts.Config.INDEXER_CURRENT_LIMIT);
        indexer.setInverted(true);

        VirtualPD.registerMotor(rightMotor::getStatorCurrent, "FlyWheel Right", "Shooter");
        VirtualPD.registerMotor(leftMotor::getStatorCurrent, "FlyWheel Left", "Shooter");
        VirtualPD.registerMotor(indexer::getStatorCurrent, "Turret Indexer", "Shooter");
    }

    @Override
    public void processInputs(FlyWheelIOInputsAutoLogged inputs) {
        double averageRPM = (rightMotor.getRPM() + leftMotor.getRPM()) / 2.0;

        inputs.averageSpeed = averageRPM;
        inputs.speedSetpoint = setpointRPM;
        inputs.atSetpoint = setpointRPM > 0.0
                && Math.abs(setpointRPM - averageRPM) <= FlyWheelConsts.Setpoint.TOLERANCE_RPM;
        inputs.isShooting = averageRPM > 1.0;

        rightMotor.updateInputs(rightInputs);
        leftMotor.updateInputs(leftInputs);
        indexer.updateInputs(indexerInputs);
    }

    @Override
    public void setFlyWheelSpeed(double speed) {
        this.setpointRPM = speed;
        rightMotor.setVelocitySetpoint(speed / SECONDS_PER_MINUTE);
    }

    @Override
    public void stop() {
        this.setpointRPM = 0.0;
        rightMotor.stop();
    }
}
