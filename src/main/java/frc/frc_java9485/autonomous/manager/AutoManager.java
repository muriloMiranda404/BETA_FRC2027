package frc.frc_java9485.autonomous;

import java.util.Objects;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.frc_java9485.utils.VirtualSubsystem;


public class AutoManager extends VirtualSubsystem {

    private static final String LOG_KEY = "Auto/";


    private static final double POSE_WARNING_DISTANCE_M = 0.25;


    private static final double POSE_WARNING_ANGLE_DEG = 8.0;

    private final AutoChooser chooser;
    private final Supplier<Pose2d> poseSupplier;

    private String builtAutoName = null;
    private Command builtCommand = Commands.none();
    private Pose2d builtStartingPose = null;

    private final Timer runTimer = new Timer();
    private boolean wasRunningAuto = false;

    private final Alert buildFailedAlert =
            new Alert("Failed to build the selected autonomous routine.", AlertType.kError);
    private final Alert poseMismatchAlert =
            new Alert("Robot is not at the selected auto's starting pose.", AlertType.kWarning);

    public AutoManager(AutoChooser chooser, Supplier<Pose2d> poseSupplier) {
        this.chooser = chooser;
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {
        String selected = chooser.getSelectedOption();


        if (!Objects.equals(selected, builtAutoName) && DriverStation.isDisabled()) {
            build(selected);
        }

        updatePoseCheck();
        updateRunTelemetry();

        Logger.recordOutput(LOG_KEY + "Selected", selected);
        Logger.recordOutput(LOG_KEY + "Built", Objects.equals(selected, builtAutoName));
    }

    @Override
    public void periodicAfterScheduler() {

    }


    public Command getAutonomousCommand() {
        String selected = chooser.getSelectedOption();
        if (!Objects.equals(selected, builtAutoName)) {
            DriverStation.reportWarning(
                    "[Auto] Building \"" + selected + "\" at autonomousInit; it was not pre-built.", false);
            build(selected);
        }
        return builtCommand;
    }


    public Pose2d getStartingPose() {
        return builtStartingPose;
    }



    private void build(String autoName) {
        builtAutoName = autoName;

        if (autoName == null || autoName.equals(AutoChooser.NONE_OPTION)) {
            builtCommand = Commands.none();
            builtStartingPose = null;
            buildFailedAlert.set(false);
            return;
        }

        try {
            double startTime = Timer.getFPGATimestamp();
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            builtCommand = auto;
            builtStartingPose = auto.getStartingPose();
            buildFailedAlert.set(false);

            System.out.printf(
                    "[Auto] Built \"%s\" in %.0f ms%n", autoName, (Timer.getFPGATimestamp() - startTime) * 1000.0);
        } catch (RuntimeException e) {

            String message = "[Auto] Failed to build \"" + autoName + "\": " + e.getMessage();
            DriverStation.reportError(message, false);
            buildFailedAlert.setText(message);
            buildFailedAlert.set(true);

            builtCommand = Commands.none();
            builtStartingPose = null;
        }
    }



    private void updatePoseCheck() {
        if (builtStartingPose == null || !DriverStation.isDisabled()) {
            poseMismatchAlert.set(false);
            return;
        }

        Pose2d expected = allianceAdjustedStartingPose();
        Pose2d actual = poseSupplier.get();

        double translationError = expected.getTranslation().getDistance(actual.getTranslation());
        double rotationError = Math.abs(
                expected.getRotation().minus(actual.getRotation()).getDegrees());

        Logger.recordOutput(LOG_KEY + "StartingPose", expected);
        Logger.recordOutput(LOG_KEY + "StartingPoseErrorM", translationError);
        Logger.recordOutput(LOG_KEY + "StartingPoseErrorDeg", rotationError);

        boolean misplaced =
                translationError > POSE_WARNING_DISTANCE_M || rotationError > POSE_WARNING_ANGLE_DEG;
        if (misplaced) {
            poseMismatchAlert.setText(String.format(
                    "Robot is %.2f m / %.0f deg off the start of \"%s\".",
                    translationError, rotationError, builtAutoName));
        }
        poseMismatchAlert.set(misplaced);
    }


    private Pose2d allianceAdjustedStartingPose() {
        boolean isRed = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false);
        return isRed ? FlippingUtil.flipFieldPose(builtStartingPose) : builtStartingPose;
    }




    private void updateRunTelemetry() {
        boolean runningAuto = DriverStation.isAutonomousEnabled();

        if (runningAuto && !wasRunningAuto) {
            runTimer.restart();
            System.out.println("[Auto] Started \"" + builtAutoName + "\"");
        } else if (!runningAuto && wasRunningAuto) {
            runTimer.stop();
            Logger.recordOutput(LOG_KEY + "LastRunSeconds", runTimer.get());
            System.out.printf("[Auto] \"%s\" ended after %.2f s%n", builtAutoName, runTimer.get());
        }

        if (runningAuto) {
            Logger.recordOutput(LOG_KEY + "ElapsedSeconds", runTimer.get());
            Logger.recordOutput(LOG_KEY + "CommandFinished", builtCommand.isFinished());
        }

        wasRunningAuto = runningAuto;
    }
}
