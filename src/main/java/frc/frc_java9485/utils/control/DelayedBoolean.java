package frc.frc_java9485.utils.control;


public class DelayedBoolean {

    private final double delaySeconds;

    private boolean lastValue;
    private double transitionTimestamp;


    public DelayedBoolean(double timestampSeconds, double delaySeconds) {
        this.transitionTimestamp = timestampSeconds;
        this.delaySeconds = delaySeconds;
        this.lastValue = false;
    }


    public boolean update(double timestampSeconds, boolean value) {

        if (value && !lastValue) {
            transitionTimestamp = timestampSeconds;
        }

        boolean result = value && (timestampSeconds - transitionTimestamp) > delaySeconds;

        lastValue = value;
        return result;
    }


    public void reset(double timestampSeconds) {
        this.transitionTimestamp = timestampSeconds;
        this.lastValue = false;
    }

    public double getDelaySeconds() {
        return delaySeconds;
    }
}
