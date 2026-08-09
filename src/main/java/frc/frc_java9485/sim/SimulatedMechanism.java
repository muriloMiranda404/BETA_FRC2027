package frc.frc_java9485.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;


public abstract class SimulatedMechanism {

    private final double gearing;
    private double lastTimestampSec;


    protected SimulatedMechanism(double gearing) {
        if (gearing <= 0.0) {
            throw new IllegalArgumentException("Gearing must be positive, got " + gearing);
        }
        this.gearing = gearing;
        this.lastTimestampSec = Timer.getFPGATimestamp();
    }

    public double getGearing() {
        return gearing;
    }


    public double mechanismToRotor(double mechanism) {
        return mechanism * gearing;
    }


    public double rotorToMechanism(double rotor) {
        return rotor / gearing;
    }


    public void setVoltageClamped(double volts) {
        double limit = RobotController.getBatteryVoltage();

        if (!Double.isFinite(limit) || limit <= 0.0) {
            limit = 12.0;
        }
        setVoltage(MathUtil.clamp(volts, -limit, limit));
    }


    public void simulate() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestampSec;
        lastTimestampSec = now;


        if (dt <= 0.0 || dt > 1.0) {
            dt = 0.02;
        }
        update(dt);
    }




    public abstract void setVoltage(double volts);


    public abstract double getPosition();


    public abstract double getVelocity();


    public abstract double getCurrentAmps();


    public abstract void setState(double position, double velocity);


    protected abstract void update(double dtSeconds);
}
