package frc.robot.subsystems.mechanism.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.frc_java9485.utils.VirtualSubsystem;
import frc.frc_java9485.utils.logger.LoggedTunableNumber;
import frc.robot.RobotState;


public class ShotCalculator extends VirtualSubsystem {

    private static final double LOOP_PERIOD_SEC = 0.02;


    private static final int MAX_ITERATIONS = 10;


    private static final double CONVERGENCE_THRESHOLD_SEC = 5e-4;


    private static LoggedTunableNumber mechanismLatency;

    private static LoggedTunableNumber mechanismLatency() {
        if (mechanismLatency == null) {
            mechanismLatency = new LoggedTunableNumber(
                    "ShotCalc/MechanismLatencySec", TurretConsts.MotionComp.MECHANISM_LATENCY_SEC);
        }
        return mechanismLatency;
    }


    public record ShotSolution(
            double fieldHeadingDeg,
            double turretRelativeAngleDeg,
            double hoodPosition,
            double flywheelRPM,
            double compensatedDistanceM,
            double timeOfFlightSec,
            boolean converged) {}




    public static double lookaheadSeconds() {
        return LOOP_PERIOD_SEC + mechanismLatency().get();
    }


    public static ShotSolution solve(
            Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, Translation3d target, double latencySec) {

        double vx = fieldRelativeSpeeds.vxMetersPerSecond;
        double vy = fieldRelativeSpeeds.vyMetersPerSecond;

        double headingRad = robotPose.getRotation().getRadians();


        double shooterX = robotPose.getX()
                + Math.cos(headingRad) * TurretConsts.MotionComp.SHOOTER_FORWARD_OFFSET_M
                + vx * latencySec;
        double shooterY = robotPose.getY()
                + Math.sin(headingRad) * TurretConsts.MotionComp.SHOOTER_FORWARD_OFFSET_M
                + vy * latencySec;

        double tx = target.getX();
        double ty = target.getY();


        double straightDistance = Math.hypot(tx - shooterX, ty - shooterY);
        double tof = TurretConsts.ShotModel.DISTANCE_TO_TOF.get(straightDistance);
        double compensatedDistance = straightDistance;
        boolean converged = false;

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double cdx = tx - shooterX - vx * tof;
            double cdy = ty - shooterY - vy * tof;
            compensatedDistance = Math.hypot(cdx, cdy);

            double newTof = TurretConsts.ShotModel.DISTANCE_TO_TOF.get(compensatedDistance);
            if (Math.abs(newTof - tof) < CONVERGENCE_THRESHOLD_SEC) {
                tof = newTof;
                converged = true;
                break;
            }
            tof = newTof;
        }


        double virtualTargetX = tx - vx * tof;
        double virtualTargetY = ty - vy * tof;

        double fieldHeadingRad = Math.atan2(virtualTargetY - shooterY, virtualTargetX - shooterX);
        double fieldHeadingDeg = Math.toDegrees(fieldHeadingRad);
        double turretRelativeDeg = MathUtil.inputModulus(
                fieldHeadingDeg - robotPose.getRotation().getDegrees(), -180.0, 180.0);

        double hoodPosition = TurretConsts.ShotModel.DISTANCE_TO_HOOD_POSITION.get(compensatedDistance);
        double flywheelRPM = TurretConsts.ShotModel.DISTANCE_TO_RPM.get(compensatedDistance);


        return new ShotSolution(
                fieldHeadingDeg,
                turretRelativeDeg,
                hoodPosition,
                flywheelRPM,
                compensatedDistance,
                tof,
                converged);
    }


    public static ShotSolution solve(Translation3d target) {
        RobotState state = RobotState.getInstance();
        return solve(
                state.getFieldToRobotPose(),
                state.getMeasuredFieldRelativeChassisSpeeds(),
                target,
                lookaheadSeconds());
    }



    private static ShotCalculator instance;

    public static ShotCalculator getInstance() {
        if (instance == null) {
            instance = new ShotCalculator();
        }
        return instance;
    }

    private Translation3d target = null;
    private ShotSolution latest = null;

    private final LinearFilter hoodFilter =
            LinearFilter.movingAverage(TurretConsts.MotionComp.SMOOTHING_WINDOW_LOOPS);
    private final LinearFilter headingFilter =
            LinearFilter.movingAverage(TurretConsts.MotionComp.SMOOTHING_WINDOW_LOOPS);

    private ShotCalculator() {}


    public void setTarget(Translation3d target) {
        this.target = target;
    }


    public ShotSolution getLatestSolution() {
        return latest;
    }

    @Override
    public void periodic() {
        if (target == null) {
            latest = null;
            return;
        }

        ShotSolution raw = solve(target);


        double smoothedHood = hoodFilter.calculate(raw.hoodPosition());
        double smoothedHeading = headingFilter.calculate(raw.fieldHeadingDeg());
        double smoothedTurretRel = MathUtil.inputModulus(
                smoothedHeading - RobotState.getInstance().getFieldToRobotPose().getRotation().getDegrees(),
                -180.0, 180.0);

        latest = new ShotSolution(
                smoothedHeading,
                smoothedTurretRel,
                smoothedHood,
                raw.flywheelRPM(),
                raw.compensatedDistanceM(),
                raw.timeOfFlightSec(),
                raw.converged());

        ChassisSpeeds speeds = RobotState.getInstance().getMeasuredFieldRelativeChassisSpeeds();
        Translation2d virtualTarget = new Translation2d(
                target.getX() - speeds.vxMetersPerSecond * latest.timeOfFlightSec(),
                target.getY() - speeds.vyMetersPerSecond * latest.timeOfFlightSec());

        Logger.recordOutput("ShotCalc/FieldHeadingDeg", latest.fieldHeadingDeg());
        Logger.recordOutput("ShotCalc/TurretRelativeDeg", latest.turretRelativeAngleDeg());
        Logger.recordOutput("ShotCalc/HoodPosition", latest.hoodPosition());
        Logger.recordOutput("ShotCalc/FlywheelRPM", latest.flywheelRPM());
        Logger.recordOutput("ShotCalc/CompensatedDistanceM", latest.compensatedDistanceM());
        Logger.recordOutput("ShotCalc/TimeOfFlightSec", latest.timeOfFlightSec());
        Logger.recordOutput("ShotCalc/Converged", latest.converged());
        Logger.recordOutput("ShotCalc/VirtualTarget", virtualTarget);
    }

    @Override
    public void periodicAfterScheduler() {

    }
}
