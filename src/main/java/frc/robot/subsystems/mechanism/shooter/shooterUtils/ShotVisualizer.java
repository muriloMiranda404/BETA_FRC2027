
package frc.robot.subsystems.mechanism.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.frc_java9485.utils.VirtualSubsystem;
import frc.robot.RobotState;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator.ShotSolution;


public class ShotVisualizer extends VirtualSubsystem {

    private static final String LOG_KEY = "ShotVisualizer/";

    private static final double GRAVITY_MPS2 = 9.81;


    private static final int ARC_SAMPLES = 20;

    private static ShotVisualizer instance;

    public static ShotVisualizer getInstance() {
        if (instance == null) {
            instance = new ShotVisualizer();
        }
        return instance;
    }

    private Translation3d target = null;

    private ShotVisualizer() {}


    public void setTarget(Translation3d target) {
        this.target = target;
    }

    @Override
    public void periodic() {
        ShotSolution solution = ShotCalculator.getInstance().getLatestSolution();
        if (solution == null || target == null || !Double.isFinite(solution.timeOfFlightSec())) {
            Logger.recordOutput(LOG_KEY + "Arc", new Pose3d[0]);
            return;
        }

        Logger.recordOutput(LOG_KEY + "Arc", buildArc(solution));
        Logger.recordOutput(LOG_KEY + "Target", new Pose3d(target, Rotation3d.kZero));
    }


    private Pose3d[] buildArc(ShotSolution solution) {
        var robotPose = RobotState.getInstance().getFieldToRobotPose();
        double headingRad = Math.toRadians(solution.fieldHeadingDeg());

        double startX = robotPose.getX()
                + Math.cos(headingRad) * TurretConsts.MotionComp.SHOOTER_FORWARD_OFFSET_M;
        double startY = robotPose.getY()
                + Math.sin(headingRad) * TurretConsts.MotionComp.SHOOTER_FORWARD_OFFSET_M;
        double startZ = TurretConsts.Config.ROBOT_TO_TURRET_TRANSFORM.getZ();

        double tof = Math.max(solution.timeOfFlightSec(), 1e-3);
        double horizontalDistance = solution.compensatedDistanceM();
        double horizontalSpeed = horizontalDistance / tof;


        double verticalSpeed = ((target.getZ() - startZ) + 0.5 * GRAVITY_MPS2 * tof * tof) / tof;

        Pose3d[] arc = new Pose3d[ARC_SAMPLES];
        for (int i = 0; i < ARC_SAMPLES; i++) {
            double t = tof * i / (ARC_SAMPLES - 1.0);
            double travelled = horizontalSpeed * t;

            double x = startX + Math.cos(headingRad) * travelled;
            double y = startY + Math.sin(headingRad) * travelled;
            double z = startZ + verticalSpeed * t - 0.5 * GRAVITY_MPS2 * t * t;


            double verticalRate = verticalSpeed - GRAVITY_MPS2 * t;
            double pitch = -Math.atan2(verticalRate, horizontalSpeed);

            arc[i] = new Pose3d(new Translation3d(x, y, z), new Rotation3d(0.0, pitch, headingRad));
        }
        return arc;
    }

    @Override
    public void periodicAfterScheduler() {

    }
}
