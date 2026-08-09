package frc.frc_java9485.autonomous;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;


public class AutoChooser {


    public static final String NONE_OPTION = "None";

    private final List<String> autos = new ArrayList<>();
    private final LoggedDashboardChooser<String> chooser;

    private final Alert noAutosAlert =
            new Alert("No PathPlanner autos found in the deploy directory.", AlertType.kWarning);
    private final Alert readFailedAlert =
            new Alert("Failed to read the PathPlanner autos directory.", AlertType.kError);


    public AutoChooser(String chooserName, String defaultOption) {
        if (chooserName == null) {
            throw new IllegalArgumentException("The chooser name cannot be null");
        }

        Path deployDirectory = Paths.get(Filesystem.getDeployDirectory().getPath());
        Path autosDir = deployDirectory.resolve("pathplanner/autos");

        chooser = new LoggedDashboardChooser<>(chooserName);

        loadAutoNames(autosDir);


        String resolvedDefault =
                (defaultOption != null && autos.contains(defaultOption)) ? defaultOption : NONE_OPTION;
        chooser.addDefaultOption(resolvedDefault, resolvedDefault);

        if (!resolvedDefault.equals(NONE_OPTION)) {
            chooser.addOption(NONE_OPTION, NONE_OPTION);
        }
        for (String auto : autos) {
            if (!auto.equals(resolvedDefault)) {
                chooser.addOption(auto, auto);
            }
        }
    }

    private void loadAutoNames(Path autosDir) {
        if (!Files.isDirectory(autosDir)) {
            String message = "[Auto] PathPlanner autos directory not found: " + autosDir;
            DriverStation.reportError(message, false);
            readFailedAlert.setText(message);
            readFailedAlert.set(true);
            return;
        }

        try (Stream<Path> files = Files.list(autosDir)) {
            files.filter(f -> f.toString().endsWith(".auto"))
                    .map(f -> f.getFileName().toString().replace(".auto", ""))
                    .sorted()
                    .forEach(autos::add);
        } catch (IOException e) {
            String message = "[Auto] Failed to list " + autosDir + ": " + e.getMessage();
            DriverStation.reportError(message, false);
            readFailedAlert.setText(message);
            readFailedAlert.set(true);
            return;
        }

        if (autos.isEmpty()) {
            DriverStation.reportWarning("[Auto] No .auto files found in " + autosDir, false);
            noAutosAlert.set(true);
        } else {
            System.out.println("[Auto] Loaded " + autos.size() + " autos: " + String.join(", ", autos));
        }
    }

    public LoggedDashboardChooser<String> getChooser() {
        return chooser;
    }


    public String getSelectedOption() {
        String selected = chooser.get();
        return (selected == null || selected.isBlank()) ? NONE_OPTION : selected;
    }


    public boolean hasSelection() {
        return !getSelectedOption().equals(NONE_OPTION);
    }


    public List<String> getAvailableAutos() {
        return List.copyOf(autos);
    }
}
