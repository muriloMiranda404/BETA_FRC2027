package frc.robot.subsystems.mechanism.shooter.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;

public class TurretCalculator {

    private TurretCalculator() {}

    /**
     * Calcula o ângulo do turret relativo ao robô para apontar para o target.
     * @param robotPose pose atual do robô no field
     * @param target posição 3d do alvo no field
     * @return ângulo em graus relativo ao robô
     */
    public static double calculateTurretAngle(Pose2d robotPose, Translation3d target) {
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();

        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        return fieldAngleDeg - robotHeadingDeg;
    }

    public record ShotParameters(double hoodAngleDeg, double flywheelRPM) {

        public ShotParameters {
            hoodAngleDeg = Math.max(0.0, Math.min(3.5, hoodAngleDeg));
        }
    }
}
