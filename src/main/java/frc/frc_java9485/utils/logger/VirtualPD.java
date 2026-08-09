package frc.frc_java9485.utils.logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;


public class VirtualPD {

    private static final List<DoubleSupplier> currentSuppliers = new ArrayList<>();
    private static final List<String> groups = new ArrayList<>();
    private static final List<String> names = new ArrayList<>();

    private static final Map<String, Energy> groupEnergyTotals = new HashMap<>();
    private static Energy totalEnergy = Joules.zero();

    private static double lastTimestamp = -1.0;

    private VirtualPD() {}


    public static void registerMotor(DoubleSupplier currentSupplier, String name, String group) {
        currentSuppliers.add(currentSupplier);
        names.add(name);
        groups.add(group);
        groupEnergyTotals.putIfAbsent(group, Joules.zero());
    }


    public static void logAll() {
        if (currentSuppliers.isEmpty()) {
            return;
        }

        double now = Timer.getFPGATimestamp();

        double dtSeconds = lastTimestamp < 0.0 ? 0.02 : now - lastTimestamp;
        lastTimestamp = now;

        double batteryVolts = RobotController.getBatteryVoltage();
        double totalAmps = 0.0;
        Map<String, Double> groupCurrentTotals = new HashMap<>();

        for (int i = 0; i < currentSuppliers.size(); i++) {
            double amps = currentSuppliers.get(i).getAsDouble();
            if (!Double.isFinite(amps)) {
                continue;
            }

            totalAmps += amps;

            String group = groups.get(i);
            groupCurrentTotals.merge(group, amps, Double::sum);

            Energy energy = Joules.of(amps * batteryVolts * dtSeconds);
            groupEnergyTotals.merge(group, energy, Energy::plus);
            totalEnergy = totalEnergy.plus(energy);

            Logger.recordOutput("VirtualPD/Motors/" + names.get(i) + "/Amps", amps);
        }

        Logger.recordOutput("VirtualPD/TotalAmps", totalAmps);
        Logger.recordOutput("VirtualPD/TotalEnergyJoules", totalEnergy.in(Joules));

        groupCurrentTotals.forEach((group, amps) -> Logger.recordOutput("VirtualPD/Current/" + group, amps));
        groupEnergyTotals.forEach(
                (group, energy) -> Logger.recordOutput("VirtualPD/Energy/" + group, energy.in(Joules)));
    }


    public static double getTotalAmps() {
        return currentSuppliers.stream()
                .mapToDouble(DoubleSupplier::getAsDouble)
                .filter(Double::isFinite)
                .sum();
    }

    public static Energy getTotalEnergy() {
        return totalEnergy;
    }

    public static Energy getGroupEnergy(String group) {
        return groupEnergyTotals.getOrDefault(group, Joules.zero());
    }


    public static void resetEnergy() {
        totalEnergy = Joules.zero();
        groupEnergyTotals.replaceAll((group, energy) -> Joules.zero());
        lastTimestamp = -1.0;
    }


    static double joulesFor(double amps, double volts, double seconds) {
        return Amps.of(amps).baseUnitMagnitude() * volts * Seconds.of(seconds).baseUnitMagnitude();
    }
}
