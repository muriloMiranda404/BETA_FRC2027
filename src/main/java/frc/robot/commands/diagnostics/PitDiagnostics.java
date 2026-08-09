package frc.robot.commands.diagnostics;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanism.SuperStructure;


public class PitDiagnostics {

    private static final String LOG_KEY = "PitDiagnostics/";


    private static final double EXERCISE_SECONDS = 1.5;


    private static final double MIN_TURRET_MOVEMENT_DEG = 2.0;
    private static final double MIN_HOOD_MOVEMENT = 0.2;
    private static final double MIN_INTAKE_MOVEMENT = 2.0;
    private static final double MIN_CLIMBER_MOVEMENT = 1.0;

    private static final Alert resultAlert =
            new Alert("Diagnostico de pit nao executado.", AlertType.kInfo);

    private PitDiagnostics() {}


    public record Result(String mechanism, boolean passed, double movement, double required) {

        @Override
        public String toString() {
            return String.format("%s: %s (moveu %.2f, minimo %.2f)",
                    mechanism, passed ? "OK" : "FALHOU", movement, required);
        }
    }


    public static Command runAll(SuperStructure superStructure, ClimberSubsystem climber) {
        List<Result> results = new ArrayList<>();

        return Commands.sequence(
                        Commands.runOnce(() -> {
                            results.clear();
                            Logger.recordOutput(LOG_KEY + "Running", true);
                        }),
                        exercise("Turret", results,
                                superStructure::getTurretAngleDeg, MIN_TURRET_MOVEMENT_DEG,
                                superStructure.prepare()),
                        exercise("Hood", results,
                                superStructure::getHoodPosition, MIN_HOOD_MOVEMENT,
                                superStructure.prepare()),
                        exercise("Intake", results,
                                superStructure::getIntakePivotPosition, MIN_INTAKE_MOVEMENT,
                                superStructure.collect()),
                        exercise("Climber", results,
                                climber::getPosition, MIN_CLIMBER_MOVEMENT,
                                climber.extend()),
                        Commands.runOnce(() -> publish(results)))
                .withName("PitDiagnostics");
    }


    private static Command exercise(
            String mechanism,
            List<Result> results,
            DoubleSupplier measurement,
            double minimumMovement,
            Command action) {

        double[] startPosition = {0.0};

        return Commands.runOnce(() -> startPosition[0] = measurement.getAsDouble())
                .andThen(action.withTimeout(EXERCISE_SECONDS))
                .andThen(Commands.runOnce(() -> {
                    double movement = Math.abs(measurement.getAsDouble() - startPosition[0]);
                    Result result = new Result(mechanism, movement >= minimumMovement, movement, minimumMovement);
                    results.add(result);

                    Logger.recordOutput(LOG_KEY + mechanism + "/Passed", result.passed());
                    Logger.recordOutput(LOG_KEY + mechanism + "/Movement", movement);
                    System.out.println("[PitDiagnostics] " + result);
                }));
    }

    private static void publish(List<Result> results) {
        Map<String, Boolean> summary = new LinkedHashMap<>();
        List<String> failures = new ArrayList<>();

        for (Result result : results) {
            summary.put(result.mechanism(), result.passed());
            if (!result.passed()) {
                failures.add(result.mechanism());
            }
        }

        Logger.recordOutput(LOG_KEY + "Running", false);
        Logger.recordOutput(LOG_KEY + "AllPassed", failures.isEmpty());
        Logger.recordOutput(LOG_KEY + "Failures", String.join(", ", failures));

        if (failures.isEmpty()) {
            resultAlert.setText("Diagnostico de pit: todos os mecanismos OK (" + summary.size() + ").");
        } else {
            resultAlert.setText("Diagnostico de pit FALHOU em: " + String.join(", ", failures));
        }
        resultAlert.set(true);

        System.out.println("[PitDiagnostics] "
                + (failures.isEmpty()
                        ? "todos os " + summary.size() + " mecanismos OK"
                        : "falhas em " + String.join(", ", failures)));
    }
}
