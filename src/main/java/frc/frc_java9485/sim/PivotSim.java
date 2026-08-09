package frc.frc_java9485.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class PivotSim extends SimulatedMechanism {

    private final SingleJointedArmSim sim;

    public PivotSim(Config config) {
        super(config.gearing);
        this.sim = new SingleJointedArmSim(
                config.motor,
                config.gearing,
                config.momentOfInertiaKgM2,
                config.armLengthMeters,
                config.minAngleRad,
                config.maxAngleRad,
                config.simulateGravity,
                config.startingAngleRad);
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public double getPosition() {
        return sim.getAngleRads();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityRadPerSec();
    }

    @Override
    public double getCurrentAmps() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public void setState(double positionRad, double velocityRadPerSec) {
        sim.setState(positionRad, velocityRadPerSec);
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


    public static class Config {
        public DCMotor motor = DCMotor.getKrakenX60(1);
        public double gearing = 1.0;
        public double momentOfInertiaKgM2 = 0.01;
        public double armLengthMeters = 0.3;
        public double minAngleRad = -Math.PI;
        public double maxAngleRad = Math.PI;
        public boolean simulateGravity = true;
        public double startingAngleRad = 0.0;

        public Config withMotor(DCMotor motor) {
            this.motor = motor;
            return this;
        }

        public Config withGearing(double gearing) {
            this.gearing = gearing;
            return this;
        }

        public Config withMomentOfInertia(double momentOfInertiaKgM2) {
            this.momentOfInertiaKgM2 = momentOfInertiaKgM2;
            return this;
        }

        public Config withArmLength(double armLengthMeters) {
            this.armLengthMeters = armLengthMeters;
            return this;
        }

        public Config withHardStops(double minAngleRad, double maxAngleRad) {
            this.minAngleRad = minAngleRad;
            this.maxAngleRad = maxAngleRad;
            return this;
        }

        public Config withGravity(boolean simulateGravity) {
            this.simulateGravity = simulateGravity;
            return this;
        }

        public Config withStartingAngle(double startingAngleRad) {
            this.startingAngleRad = startingAngleRad;
            return this;
        }
    }
}
