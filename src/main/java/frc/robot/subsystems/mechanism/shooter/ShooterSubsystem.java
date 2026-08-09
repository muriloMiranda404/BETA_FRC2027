package frc.robot.subsystems.mechanism.shooter;


import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.mechanisms.shooter.ShotVerifierConsts;
import frc.frc_java9485.constants.utils.FieldElementsConst;
import frc.robot.RobotState;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator.ShotSolution;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIO;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelSubsystem;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIO;
import frc.robot.subsystems.mechanism.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIO;
import frc.robot.subsystems.mechanism.shooter.turret.TurretSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final FlyWheelSubsystem flyWheel;


    @AutoLogOutput
    private Translation3d currentTarget;

    @AutoLogOutput
    private SystemState currentState = SystemState.OFF;
    private ShooterWantedState wantedState = ShooterWantedState.OFF;

    private double passingRPM = 2000.0;

    private final Debouncer verifiedDebouncer =
            new Debouncer(ShotVerifierConsts.VERIFIED_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
    private ShotVerifier.Verification lastVerification =
            ShotVerifier.Verification.rejected(ShotVerifier.Rejection.NO_SOLUTION);
    private boolean verifiedDebounced = false;

    private double characterizationHoodPosition = 0.0;
    private double characterizationRPM = 0.0;

    public ShooterSubsystem(TurretIO turretIO, HoodIO hoodIO, FlyWheelIO flyWheelIO) {

        this.flyWheel = new FlyWheelSubsystem(flyWheelIO);
        this.hood = new HoodSubsystem(hoodIO);
        this.turret = new TurretSubsystem(turretIO);

        this.currentTarget = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red
                ? FieldElementsConst.HubMeansured.HUB_RED
                : FieldElementsConst.HubMeansured.HUB_BLUE)
            .orElse(FieldElementsConst.HubMeansured.HUB_BLUE);


        ShotCalculator.getInstance().setTarget(currentTarget);
        ShotVisualizer.getInstance().setTarget(currentTarget);
    }

    @Override
    public void periodic() {
        currentState = handleTransition();
        executeActions();


        turret.update();
        hood.update();
        flyWheel.update();

        updateShotVerification();
    }


    private void updateShotVerification() {
        RobotState state = RobotState.getInstance();

        lastVerification = switch (currentState) {
            case AIMING -> ShotVerifier.verifyHubShot(
                    ShotCalculator.getInstance().getLatestSolution(),
                    state.getFieldToRobotPose().getRotation().getDegrees(),
                    state.getYawRateRadPerSec(),
                    state.getPitchDegrees(),
                    state.getRollDegrees());
            case PASSING -> ShotVerifier.verifyPassShot(
                    ShotCalculator.getInstance().getLatestSolution(),
                    state.getFieldToRobotPose().getRotation().getDegrees(),
                    state.getYawRateRadPerSec(),
                    state.getPitchDegrees(),
                    state.getRollDegrees());

            case CHARACTERIZING -> ShotVerifier.Verification.OK;
            case OFF -> ShotVerifier.Verification.rejected(ShotVerifier.Rejection.NO_SOLUTION);
        };

        verifiedDebounced = verifiedDebouncer.calculate(lastVerification.verified());
    }

    public void setWantedState(ShooterWantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setTarget(Translation3d target) {
        this.currentTarget = target;
        ShotVisualizer.getInstance().setTarget(target);
        ShotCalculator.getInstance().setTarget(target);
    }

    public Translation3d getCurrentTarget() {
        return currentTarget;
    }

    public void setPassingRPM(double rpm) {
        this.passingRPM = rpm;
    }


    public void setCharacterizationOutputs(double hoodPosition, double flywheelRPM) {
        this.characterizationHoodPosition = hoodPosition;
        this.characterizationRPM = flywheelRPM;
    }


    public double getTurretAngleDeg() {
        return turret.getMeasuredPosition();
    }


    public double getHoodPosition() {
        return hood.getMeasuredPosition();
    }


    @AutoLogOutput
    public boolean mechanismsAtSetpoint() {
        return turret.atSetpoint() && hood.atSetpoint() && flyWheel.atSetpoint();
    }


    @AutoLogOutput
    public ShotVerifier.Rejection getShotRejection() {
        return lastVerification.rejection();
    }


    @AutoLogOutput
    public boolean isReadyToShoot() {
        return mechanismsAtSetpoint() && verifiedDebounced;
    }

    private SystemState handleTransition() {
        return switch (wantedState) {
            case AIMING -> SystemState.AIMING;
            case PASSING -> SystemState.PASSING;
            case CHARACTERIZING -> SystemState.CHARACTERIZING;
            case OFF -> SystemState.OFF;
        };
    }

    private void executeActions() {
        switch (currentState) {
            case AIMING -> {
                ShotSolution shot = currentShotSolution();


                turret.setAimingSetpoint(shot.turretRelativeAngleDeg());
                turret.setWantedState(TurretSubsystem.WantedState.AIMING);

                hood.setHoodPosition(shot.hoodPosition());
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
            case CHARACTERIZING -> {

                turret.setAimingSetpoint(currentShotSolution().turretRelativeAngleDeg());
                turret.setWantedState(TurretSubsystem.WantedState.AIMING);

                hood.setHoodPosition(characterizationHoodPosition);
                hood.setWantedState(HoodSubsystem.WantedState.ANGLING);

                flyWheel.setShootingRPM(characterizationRPM);
                flyWheel.setWantedState(FlyWheelSubsystem.WantedState.SHOOTING);
            }
            case OFF -> {
                turret.setWantedState(TurretSubsystem.WantedState.OFF);
                hood.setWantedState(HoodSubsystem.WantedState.OFF);
                flyWheel.setWantedState(FlyWheelSubsystem.WantedState.OFF);
            }
        }
    }


    private ShotSolution currentShotSolution() {
        ShotSolution latest = ShotCalculator.getInstance().getLatestSolution();
        return latest != null ? latest : ShotCalculator.solve(currentTarget);
    }

    public enum SystemState {
        AIMING,
        PASSING,
        CHARACTERIZING,
        OFF
    }

    public enum ShooterWantedState {
        AIMING,
        PASSING,

        CHARACTERIZING,
        OFF
    }
}
