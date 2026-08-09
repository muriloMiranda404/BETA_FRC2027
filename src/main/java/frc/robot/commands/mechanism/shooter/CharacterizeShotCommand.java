package frc.robot.commands.mechanism.shooter;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.utils.calc.LinearLineCalculator;
import frc.frc_java9485.utils.logger.LoggedTunableNumber;
import frc.robot.RobotState;
import frc.robot.subsystems.mechanism.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanism.shooter.ShooterSubsystem.ShooterWantedState;


public class CharacterizeShotCommand extends Command {

    private static final String LOG_KEY = "ShotCharacterization/";

    private static final LoggedTunableNumber hoodPosition =
            new LoggedTunableNumber(LOG_KEY + "HoodPosition", 1.0);
    private static final LoggedTunableNumber flywheelRPM =
            new LoggedTunableNumber(LOG_KEY + "FlywheelRPM", 2500.0);


    public record Sample(double distanceM, double hoodPosition, double flywheelRPM, double airTimeSec) {}

    private final ShooterSubsystem shooter;
    private final Translation3d target;

    private final List<Sample> samples = new ArrayList<>();
    private final Timer airTimer = new Timer();
    private boolean timingShot = false;

    public CharacterizeShotCommand(ShooterSubsystem shooter, Translation3d target) {
        this.shooter = shooter;
        this.target = target;
        addRequirements(shooter);
    }


    public static CharacterizeShotCommand forCurrentTarget(ShooterSubsystem shooter) {
        return new CharacterizeShotCommand(shooter, shooter.getCurrentTarget());
    }

    @Override
    public void initialize() {
        samples.clear();
        timingShot = false;
        shooter.setWantedState(ShooterWantedState.CHARACTERIZING);
        DriverStation.reportWarning(
                "Shot characterization started. Tune " + LOG_KEY + "HoodPosition / FlywheelRPM, "
                        + "then record a sample at each distance.",
                false);
    }

    @Override
    public void execute() {
        shooter.setCharacterizationOutputs(hoodPosition.get(), flywheelRPM.get());

        Logger.recordOutput(LOG_KEY + "CurrentDistanceM", currentDistance());
        Logger.recordOutput(LOG_KEY + "SampleCount", samples.size());
        Logger.recordOutput(LOG_KEY + "TimingShot", timingShot);
    }


    public void recordSample() {
        double airTime = timingShot ? airTimer.get() : Double.NaN;
        Sample sample = new Sample(currentDistance(), hoodPosition.get(), flywheelRPM.get(), airTime);
        samples.add(sample);
        timingShot = false;

        DriverStation.reportWarning(
                String.format(
                        "Recorded sample %d: %.3f m -> hood %.3f, %.0f RPM, tof %.3f s",
                        samples.size(), sample.distanceM(), sample.hoodPosition(),
                        sample.flywheelRPM(), sample.airTimeSec()),
                false);
    }


    public void discardLastSample() {
        if (!samples.isEmpty()) {
            Sample dropped = samples.remove(samples.size() - 1);
            DriverStation.reportWarning(
                    String.format("Discarded sample at %.3f m", dropped.distanceM()), false);
        }
    }


    public void markShotReleased() {
        airTimer.restart();
        timingShot = true;
    }


    public void markShotLanded() {
        if (timingShot) {
            airTimer.stop();
        }
    }


    public Command recordSampleCommand() {
        return edu.wpi.first.wpilibj2.command.Commands.runOnce(this::recordSample)
                .ignoringDisable(true);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setWantedState(ShooterWantedState.OFF);
        printResults();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double currentDistance() {
        Translation2d robot = RobotState.getInstance().getFieldToRobotPose().getTranslation();
        return robot.getDistance(new Translation2d(target.getX(), target.getY()));
    }


    private void printResults() {
        if (samples.isEmpty()) {
            DriverStation.reportWarning("Shot characterization ended with no samples.", false);
            return;
        }

        List<Sample> sorted = new ArrayList<>(samples);
        sorted.sort(Comparator.comparingDouble(Sample::distanceM));

        StringBuilder sb = new StringBuilder();
        sb.append("\n===== Shot characterization: ").append(sorted.size()).append(" samples =====\n");
        sb.append("Paste into TurretConsts.ShotModel:\n\n");

        for (Sample s : sorted) {
            sb.append(String.format("DISTANCE_TO_RPM.put(%.3f, %.1f);%n", s.distanceM(), s.flywheelRPM()));
        }
        sb.append('\n');
        for (Sample s : sorted) {
            sb.append(String.format(
                    "DISTANCE_TO_HOOD_POSITION.put(%.3f, %.3f);%n", s.distanceM(), s.hoodPosition()));
        }

        List<Sample> timed = sorted.stream().filter(s -> Double.isFinite(s.airTimeSec())).toList();
        if (timed.isEmpty()) {
            sb.append("\n// No air times measured — DISTANCE_TO_TOF left untouched.\n");
        } else {
            sb.append('\n');
            for (Sample s : timed) {
                sb.append(String.format("DISTANCE_TO_TOF.put(%.3f, %.3f);%n", s.distanceM(), s.airTimeSec()));
            }
        }

        sb.append('\n');
        appendFit(sb, "RPM", sorted, Sample::flywheelRPM);
        appendFit(sb, "Hood", sorted, Sample::hoodPosition);
        if (timed.size() >= 2) {
            appendFit(sb, "AirTime", timed, Sample::airTimeSec);
        }
        sb.append("==================================================\n");

        System.out.print(sb);
        DriverStation.reportWarning(sb.toString(), false);
    }

    private void appendFit(
            StringBuilder sb,
            String label,
            List<Sample> data,
            java.util.function.ToDoubleFunction<Sample> valueOf) {
        if (data.size() < 2) {
            sb.append("// ").append(label).append(": not enough samples to fit\n");
            return;
        }

        List<Pair<Double, Double>> points = data.stream()
                .map(s -> Pair.of(s.distanceM(), valueOf.applyAsDouble(s)))
                .toList();
        LinearLineCalculator fit = LinearLineCalculator.bestFit(points);

        sb.append(String.format(
                "// %s best fit: %s  (R2 = %.4f)%n", label, fit, fit.rSquared(points)));
    }
}
