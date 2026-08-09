package frc.frc_java9485.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;


public class LinearSim extends SimulatedMechanism {

    private final ElevatorSim sim;
    private final double drumRadiusMeters;

    public LinearSim(Config config) {
        super(config.gearing);
        this.drumRadiusMeters = config.drumRadiusMeters;
        this.sim = new ElevatorSim(
                config.motor,
                config.gearing,
                config.carriageMassKg,
                config.drumRadiusMeters,
                config.minHeightMeters,
                config.maxHeightMeters,
                config.simulateGravity,
                config.startingHeightMeters);
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }


    @Override
    public double getPosition() {
        return sim.getPositionMeters();
    }


    @Override
    public double getVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public double getCurrentAmps() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public void setState(double positionMeters, double velocityMetersPerSec) {
        sim.setState(positionMeters, velocityMetersPerSec);
    }

    @Override
    protected void update(double dtSeconds) {
        sim.update(dtSeconds);
    }

    public boolean atLowerLimit() {
        return sim.hasHitLowerLimit();
    }

    public boolean atUpperLimit() {
        return sim.hasHitUpperLimit();
    }


    public double getPositionRotations() {
        return sim.getPositionMeters() / (2.0 * Math.PI * drumRadiusMeters);
    }


    public static class Config {
        public DCMotor motor = DCMotor.getKrakenX60(1);
        public double gearing = 1.0;
        public double carriageMassKg = 5.0;
        public double drumRadiusMeters = 0.02;
        public double minHeightMeters = 0.0;
        public double maxHeightMeters = 1.0;
        public boolean simulateGravity = true;
        public double startingHeightMeters = 0.0;

        public Config withMotor(DCMotor motor) {
            this.motor = motor;
            return this;
        }

        public Config withGearing(double gearing) {
            this.gearing = gearing;
            return this;
        }

        public Config withCarriageMass(double carriageMassKg) {
            this.carriageMassKg = carriageMassKg;
            return this;
        }

        public Config withDrumRadius(double drumRadiusMeters) {
            this.drumRadiusMeters = drumRadiusMeters;
            return this;
        }

        public Config withTravel(double minHeightMeters, double maxHeightMeters) {
            this.minHeightMeters = minHeightMeters;
            this.maxHeightMeters = maxHeightMeters;
            return this;
        }

        public Config withGravity(boolean simulateGravity) {
            this.simulateGravity = simulateGravity;
            return this;
        }

        public Config withStartingHeight(double startingHeightMeters) {
            this.startingHeightMeters = startingHeightMeters;
            return this;
        }
    }
}
