package frc.robot.subsystems.mechanism.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.utils.FieldElementsConst;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIO;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelSubsystem;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIO;
import frc.robot.subsystems.mechanism.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.TurretCalculator.ShotParameters;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIO;
import frc.robot.subsystems.mechanism.shooter.turret.TurretSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final FlyWheelSubsystem flyWheel;

    private final Supplier<Pose2d> poseSupplier;

    @AutoLogOutput
    private Translation3d currentTarget;

    @AutoLogOutput
    private SystemState currentState = SystemState.OFF;
    private ShooterWantedState wantedState = ShooterWantedState.OFF;

    private double passingRPM = 2000.0;

    public ShooterSubsystem(TurretIO turretIO, HoodIO hoodIO, FlyWheelIO flyWheelIO, Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        this.flyWheel = new FlyWheelSubsystem(flyWheelIO);
        this.hood = new HoodSubsystem(hoodIO);
        this.turret = new TurretSubsystem(turretIO, poseSupplier);

        this.currentTarget = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red
                ? FieldElementsConst.HubMeansured.HUB_RED
                : FieldElementsConst.HubMeansured.HUB_BLUE)
            .orElse(FieldElementsConst.HubMeansured.HUB_BLUE);
    }

    @Override
    public void periodic() {
        currentState = handleTransition();
        executeActions();
    }

    public void setWantedState(ShooterWantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setTarget(Translation3d target) {
        this.currentTarget = target;
        this.turret.setTarget(target);
    }

    public void setPassingRPM(double rpm) {
        this.passingRPM = rpm;
    }

    @AutoLogOutput
    public boolean isReadyToShoot() {
        return turret.atSetpoint() && hood.atSetpoint();
    }

    private SystemState handleTransition() {
        return switch (wantedState) {
            case AIMING -> SystemState.AIMING;
            case PASSING -> SystemState.PASSING;
            case OFF -> SystemState.OFF;
        };
    }

    private void executeActions() {
        switch (currentState) {
            case AIMING -> {
                ShotParameters shot = calculateShot(currentTarget);

                turret.setTarget(currentTarget);
                turret.setWantedState(TurretSubsystem.WantedState.AIMING);

                hood.setHoodAngle(shot.hoodAngleDeg());
                hood.setWantedState(HoodSubsystem.WantedState.ANGLING);

                flyWheel.setShootingRPM(shot.flywheelRPM());
                flyWheel.setWantedState(FlyWheelSubsystem.WantedState.SHOOTING);
            }
            case PASSING -> {
                turret.setWantedState(TurretSubsystem.WantedState.PASSING);
                hood.setWantedState(HoodSubsystem.WantedState.HOME);
                flyWheel.setPassingRPM(passingRPM);
                flyWheel.setWantedState(FlyWheelSubsystem.WantedState.PASSING);
            }
            case OFF -> {
                turret.setWantedState(TurretSubsystem.WantedState.OFF);
                hood.setWantedState(HoodSubsystem.WantedState.OFF);
                flyWheel.setWantedState(FlyWheelSubsystem.WantedState.OFF);
            }
        }
    }

    private ShotParameters calculateShot(Translation3d target) {
        Pose2d robotPose = poseSupplier.get();

        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double horizontalDistance = Math.sqrt(dx * dx + dy * dy);

        double heightDelta = target.getZ();

        double hoodAngleDeg = Math.toDegrees(Math.atan2(heightDelta, horizontalDistance));
        hoodAngleDeg = Math.max(0.0, Math.min(3.5, hoodAngleDeg));

        double flywheelRPM = 2000.0 + (horizontalDistance * 200.0);

        return new ShotParameters(hoodAngleDeg, flywheelRPM);
    }

    public enum SystemState {
        AIMING,
        PASSING,
        OFF
    }

    public enum ShooterWantedState {
        AIMING,
        PASSING,
        OFF
    }
}
