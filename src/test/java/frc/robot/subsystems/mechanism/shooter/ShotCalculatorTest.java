package frc.robot.subsystems.mechanism.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.mechanism.shooter.ShotCalculator.ShotSolution;
import org.junit.jupiter.api.Test;


class ShotCalculatorTest {
    private static final Translation3d TARGET = new Translation3d(8.0, 4.0, 2.0);


    @Test
    void stationary_matchesPlainGeometry() {
        Pose2d robotPose = new Pose2d(2.0, 4.0, Rotation2d.kZero);
        ShotSolution sol = ShotCalculator.solve(robotPose, new ChassisSpeeds(), TARGET, 0.0);

        double expectedDistance = Math.hypot(TARGET.getX() - 2.0, TARGET.getY() - 4.0);
        double expectedHeading =
                Math.toDegrees(Math.atan2(TARGET.getY() - 4.0, TARGET.getX() - 2.0));

        assertEquals(expectedDistance, sol.compensatedDistanceM(), 1e-3);
        assertEquals(expectedHeading, sol.fieldHeadingDeg(), 1e-6);

        assertEquals(0.0, sol.turretRelativeAngleDeg(), 1e-6);
        assertTrue(sol.converged());
    }


    @Test
    void turretRelativeAngle_accountsForRobotHeading() {
        Pose2d robotPose = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(30.0));
        ShotSolution sol = ShotCalculator.solve(robotPose, new ChassisSpeeds(), TARGET, 0.0);


        assertEquals(-30.0, sol.turretRelativeAngleDeg(), 1e-6);
    }


    @Test
    void lateralMotion_leadsTheShot() {
        Pose2d robotPose = new Pose2d(2.0, 4.0, Rotation2d.kZero);
        ChassisSpeeds movingLeft = new ChassisSpeeds(0.0, 3.0, 0.0);

        ShotSolution moving = ShotCalculator.solve(robotPose, movingLeft, TARGET, 0.0);
        ShotSolution still = ShotCalculator.solve(robotPose, new ChassisSpeeds(), TARGET, 0.0);

        assertTrue(moving.converged());

        assertTrue(moving.fieldHeadingDeg() < still.fieldHeadingDeg(),
                "expected lead toward -Y, got moving=" + moving.fieldHeadingDeg()
                        + " still=" + still.fieldHeadingDeg());
        assertTrue(moving.timeOfFlightSec() > 0.0);
    }
}
