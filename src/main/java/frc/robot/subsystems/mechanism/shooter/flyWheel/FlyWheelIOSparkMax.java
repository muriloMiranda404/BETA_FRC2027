package frc.robot.subsystems.mechanism.shooter.flyWheel;

import com.revrobotics.spark.SparkBase.ControlType;

import frc.frc_java9485.constants.mechanisms.shooter.FlyWheelConsts;
import frc.frc_java9485.motors.rev.SparkFlexMotor;
import frc.frc_java9485.motors.rev.SparkMaxMotor;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;

public class FlyWheelIOSparkMax implements FlyWheelIO{

    private final SparkFlexMotor rightMotor;
    private final SparkFlexMotor leftMotor;
    private final SparkMaxMotor indexer;

    private final SparkInputsAutoLogged rightMotorInputs;
    private final SparkInputsAutoLogged leftMotorInputs;
    private final SparkInputsAutoLogged indexerMotorInputs;

    private double shooterSetpoint;

    public FlyWheelIOSparkMax(){
        this.leftMotor = new SparkFlexMotor(FlyWheelConsts.Motors.LEFT_SHOOTER, "Left Shooter");
        this.rightMotor = new SparkFlexMotor(FlyWheelConsts.Motors.RIGHT_SHOOTER, "Right Shooter");
        this.indexer = new SparkMaxMotor(FlyWheelConsts.Motors.INDEXER, "Turret Indexer");

        this.rightMotorInputs = new SparkInputsAutoLogged();
        this.leftMotorInputs = new SparkInputsAutoLogged();
        this.indexerMotorInputs = new SparkInputsAutoLogged();

        this.configureFlyWheel();
    }

    private void configureFlyWheel(){
        this.rightMotor.setInverted(false);
        this.rightMotor.setClosedLoopPID(FlyWheelConsts.PID.Kp,
                                         FlyWheelConsts.PID.Ki,
                                         FlyWheelConsts.PID.Kd);
        this.rightMotor.setCurrentLimit(FlyWheelConsts.Config.SHOOTER_CURRENT_LIMIT);
        this.rightMotor.burnFlash();

        this.leftMotor.setInverted(true);
        this.leftMotor.followMotor(rightMotor.getDeviceId());
        this.leftMotor.setCurrentLimit(FlyWheelConsts.Config.SHOOTER_CURRENT_LIMIT);
        this.leftMotor.burnFlash();

        this.indexer.setCurrentLimit(FlyWheelConsts.Config.INDEXER_CURRENT_LIMIT);
        this.indexer.setInverted(true);
        this.indexer.burnFlash();
    }

    @Override
    public void processInputs(FlyWheelIOInputsAutoLogged inputs) {
        inputs.atSetpoit = rightMotor.atSetpoint();
        inputs.averageSpeed = (rightMotor.getRate() + leftMotor.getRate()) / 2;
        inputs.isShooting = inputs.averageSpeed > 0;
        inputs.speedSetpoint = shooterSetpoint;

        this.rightMotor.updateInputs(rightMotorInputs);
        this.leftMotor.updateInputs(leftMotorInputs);
        this.indexer.updateInputs(indexerMotorInputs);
    }

    @Override
    public void setFlyWheelSpeed(double speed) {
        this.shooterSetpoint = speed;
        this.rightMotor.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        this.shooterSetpoint = 0;
        this.rightMotor.setSetpoint(0, ControlType.kVelocity);
    }
}
