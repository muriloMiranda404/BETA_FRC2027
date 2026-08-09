package frc.robot.commands.mechanism.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.frc_java9485.utils.control.ContinuousConditionalCommand;
import frc.robot.commands.swerveUtils.commands.AimRobotToHub;
import frc.robot.commands.swerveUtils.commands.DriveIntoShotRange;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.shooter.ShotVerifier.Rejection;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class ShotCorrectionPolicy {

    private static final String LOG_KEY = "ShotCorrection/";

    private ShotCorrectionPolicy() {}


    public static Command shootWithCorrection(SwerveSubsystem swerve, SuperStructure superStructure) {
        Command correctDistance = DriveIntoShotRange.toHub(swerve);
        Command holdAim = AimRobotToHub.toHubAndHold(swerve);

        Command chassisPolicy = new ContinuousConditionalCommand(
                correctDistance,
                holdAim,
                () -> {
                    boolean distanceProblem = isDistanceProblem(superStructure.getShotRejection());
                    Logger.recordOutput(LOG_KEY + "Rejection", superStructure.getShotRejection().toString());
                    Logger.recordOutput(LOG_KEY + "Action", distanceProblem ? "REPOSITION" : "HOLD_AIM");
                    return distanceProblem;
                });

        return chassisPolicy.alongWith(superStructure.shoot()).withName("ShootWithCorrection");
    }


    public static boolean isDistanceProblem(Rejection rejection) {
        return rejection == Rejection.TOO_FAR || rejection == Rejection.TOO_CLOSE;
    }


    public static String describe(Rejection rejection) {
        return switch (rejection) {
            case NONE -> "Tiro valido";
            case NOT_AIMED -> "Chassi fora de mira";
            case TURNING_TOO_FAST -> "Girando rapido demais";
            case ROBOT_TIPPED -> "Robo inclinado";
            case TOO_FAR -> "Longe demais do hub";
            case TOO_CLOSE -> "Perto demais do hub";
            case NO_SOLUTION -> "Sem solucao de tiro";
        };
    }


    public static Command driveUntilShotIsValid(SwerveSubsystem swerve, SuperStructure superStructure) {
        return DriveIntoShotRange.toHub(swerve)
                .onlyIf(() -> isDistanceProblem(superStructure.getShotRejection()))
                .andThen(Commands.waitUntil(superStructure::isReadyToShoot))
                .withName("DriveUntilShotIsValid");
    }
}
