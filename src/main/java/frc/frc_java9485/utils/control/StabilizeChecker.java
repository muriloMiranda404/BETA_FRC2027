package frc.frc_java9485.utils.control;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;


public class StabilizeChecker {

    private static final double DEFAULT_STABILITY_TIME_SEC = 0.2;

    private final Timer timer = new Timer();
    private double stabilityTimeSec;

    public StabilizeChecker() {
        this(DEFAULT_STABILITY_TIME_SEC);
    }


    public StabilizeChecker(double stabilityTimeSec) {
        this.stabilityTimeSec = stabilityTimeSec;
        this.timer.restart();
    }


    public boolean isStableInRange(double value, double setpoint, double range) {
        return isStableInCondition(() -> Math.abs(value - setpoint) <= range);
    }


    public boolean isStableInCondition(BooleanSupplier condition) {
        if (!safeEvaluate(condition)) {
            timer.restart();
            return false;
        }
        return timer.get() >= stabilityTimeSec;
    }


    public double getStableTimeSec() {
        return timer.get();
    }


    public void reset() {
        timer.restart();
    }

    public void setStabilityTime(double stabilityTimeSec) {
        this.stabilityTimeSec = stabilityTimeSec;
    }

    public double getStabilityTime() {
        return stabilityTimeSec;
    }

    private static boolean safeEvaluate(BooleanSupplier condition) {

        try {
            return condition.getAsBoolean();
        } catch (RuntimeException e) {
            return false;
        }
    }
}
