package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class RobotStateTest {
    private static final double DELTA = 1e-9;

    @Test
    void getInstance_returnsSameSingleton() {
        assertSame(RobotState.getInstance(), RobotState.getInstance());
    }

    @Test
    void speedsReflectLastMeasurement() {
        RobotState rs = RobotState.getInstance();
        rs.addOdometryMeasurement(0.0, new Pose2d(), new ChassisSpeeds(1.0, 2.0, 3.0));

        ChassisSpeeds speeds = rs.getMeasuredFieldRelativeChassisSpeeds();
        assertEquals(1.0, speeds.vxMetersPerSecond, DELTA);
        assertEquals(2.0, speeds.vyMetersPerSecond, DELTA);
        assertEquals(3.0, speeds.omegaRadiansPerSecond, DELTA);
    }

    @Test
    void olderTimestampDoesNotOverwritePose() {
        RobotState rs = RobotState.getInstance();
        Pose2d newest = new Pose2d(5.0, 6.0, new Rotation2d());


        rs.addOdometryMeasurement(Double.MAX_VALUE, newest, new ChassisSpeeds());
        assertSame(newest, rs.getFieldToRobotPose());


        rs.addOdometryMeasurement(1.0, new Pose2d(9.0, 9.0, new Rotation2d()), new ChassisSpeeds());
        assertSame(newest, rs.getFieldToRobotPose());
    }

    @Test
    void visionObservationIsStored() {
        RobotState rs = RobotState.getInstance();
        Pose2d visionPose = new Pose2d(1.0, 1.0, new Rotation2d());
        rs.addVisionObservation(0.5, visionPose);

        assertTrue(rs.getLatestVisionPose().isPresent());
        assertSame(visionPose, rs.getLatestVisionPose().get());
    }
}
