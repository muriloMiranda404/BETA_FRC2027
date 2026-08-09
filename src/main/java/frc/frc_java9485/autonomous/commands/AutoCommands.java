package frc.frc_java9485.autonomous;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerveUtils.commands.AimRobotToHub;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;


public class AutoCommands {


    private static final double READY_TIMEOUT_S = 1.5;


    private static final double FEED_WINDOW_S = 0.8;


    private static final double SHOOT_TIMEOUT_S = 3.0;


    private static final double AIM_TIMEOUT_S = 2.0;


    private static final double COLLECT_TIMEOUT_S = 2.0;

    private static final double EJECT_TIMEOUT_S = 1.5;
    private static final double CLIMB_TIMEOUT_S = 5.0;

    private static final String LOG_KEY = "Auto/Commands/";

    private AutoCommands() {}


    public static Command shootWhenReady(SuperStructure superStructure) {
        return superStructure
                .shoot()
                .raceWith(feedWhenReady(superStructure::isReadyToShoot, "ShootWhenReady"))
                .withTimeout(SHOOT_TIMEOUT_S)
                .withName("ShootWhenReady");
    }


    public static Command aimAndShoot(SwerveSubsystem swerve, SuperStructure superStructure) {
        return AimRobotToHub.toHubAndHold(swerve)
                .alongWith(superStructure.shoot())
                .raceWith(feedWhenReady(superStructure::isReadyToShoot, "AimAndShoot"))
                .withTimeout(SHOOT_TIMEOUT_S)
                .withName("AimAndShoot");
    }


    public static Command aimAtHub(SwerveSubsystem swerve) {
        return AimRobotToHub.toHub(swerve).withTimeout(AIM_TIMEOUT_S).withName("AimAtHub");
    }


    public static Command waitForShotReady(SuperStructure superStructure) {
        return Commands.waitUntil(superStructure::isReadyToShoot)
                .withTimeout(READY_TIMEOUT_S)
                .withName("WaitForShotReady");
    }

    public static Command collect(SuperStructure superStructure) {
        return superStructure.collect().withTimeout(COLLECT_TIMEOUT_S).withName("Collect");
    }

    public static Command pass(SuperStructure superStructure) {
        return superStructure.pass().withTimeout(SHOOT_TIMEOUT_S).withName("Pass");
    }

    public static Command eject(SuperStructure superStructure) {
        return superStructure.eject().withTimeout(EJECT_TIMEOUT_S).withName("Eject");
    }

    public static Command climb(ClimberSubsystem climber) {
        return climber.retract().withTimeout(CLIMB_TIMEOUT_S).withName("Climb");
    }


    public static Command feedWhenReady(BooleanSupplier readyToShoot, String label) {
        Timer timer = new Timer();

        return Commands.runOnce(timer::restart)
                .andThen(Commands.waitUntil(readyToShoot).withTimeout(READY_TIMEOUT_S))
                .andThen(Commands.runOnce(() -> {
                    Logger.recordOutput(LOG_KEY + label + "/TimeToReadySec", timer.get());
                    Logger.recordOutput(LOG_KEY + label + "/Verified", readyToShoot.getAsBoolean());
                }))
                .andThen(Commands.waitSeconds(FEED_WINDOW_S))
                .withName(label + "/FeedWhenReady");
    }


    public static double getReadyTimeoutSeconds() {
        return READY_TIMEOUT_S;
    }


    public static double getFeedWindowSeconds() {
        return FEED_WINDOW_S;
    }
}
