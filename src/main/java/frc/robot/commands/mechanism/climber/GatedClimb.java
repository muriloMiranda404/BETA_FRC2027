package frc.robot.commands.mechanism.climber;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.utils.Rebuilt.HubTracker;
import frc.frc_java9485.utils.Rebuilt.HubTracker.Shift;
import frc.frc_java9485.utils.Rebuilt.Zones;
import frc.robot.RobotState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanism.SuperStructure;


public class GatedClimb {

    private static final String LOG_KEY = "GatedClimb/";

    private static final Alert refusedAlert =
            new Alert("Climb bloqueado pelas condicoes de seguranca.", AlertType.kWarning);

    private GatedClimb() {}


    public enum Gate {
        ALLOWED,
        NOT_ENDGAME,
        WRONG_ZONE,
        SUPERSTRUCTURE_BUSY
    }


    public static Command retract(ClimberSubsystem climber, SuperStructure superStructure) {
        return climber.retract()
                .onlyWhile(() -> publish(evaluate(superStructure)) == Gate.ALLOWED)
                .withName("GatedClimbRetract");
    }


    public static Command extend(ClimberSubsystem climber, SuperStructure superStructure) {
        return climber.extend()
                .onlyWhile(() -> publish(evaluate(superStructure)) == Gate.ALLOWED)
                .withName("GatedClimbExtend");
    }


    public static Gate evaluate(SuperStructure superStructure) {
        if (!isEndgame()) {
            return Gate.NOT_ENDGAME;
        }
        if (!isInOwnAllianceZone()) {
            return Gate.WRONG_ZONE;
        }
        if (superStructure.getCurrentState() != SuperStructure.SystemState.OFF) {
            return Gate.SUPERSTRUCTURE_BUSY;
        }
        return Gate.ALLOWED;
    }

    private static boolean isEndgame() {
        Optional<Shift> shift = HubTracker.getCurrentShift();
        return shift.isPresent() && shift.get() == Shift.ENDGAME;
    }


    private static boolean isInOwnAllianceZone() {
        Pose2d pose = RobotState.getInstance().getFieldToRobotPose();
        return Zones.BLUE_ALLIANCE_ZONE.containsPoint(pose.getTranslation())
                || Zones.RED_ALLIANCE_ZONE.containsPoint(pose.getTranslation());
    }

    private static Gate publish(Gate gate) {
        Logger.recordOutput(LOG_KEY + "Gate", gate.toString());

        boolean refused = gate != Gate.ALLOWED;
        if (refused) {
            refusedAlert.setText("Climb bloqueado: " + describe(gate));
        }
        refusedAlert.set(refused);

        return gate;
    }


    public static String describe(Gate gate) {
        return switch (gate) {
            case ALLOWED -> "Liberado";
            case NOT_ENDGAME -> "Ainda nao e o endgame";
            case WRONG_ZONE -> "Robo fora da zona de aliança";
            case SUPERSTRUCTURE_BUSY -> "Mecanismo nao esta recolhido";
        };
    }
}
