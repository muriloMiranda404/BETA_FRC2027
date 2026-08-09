package frc.frc_java9485.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;


public class RollerSim extends SimulatedMechanism {

    private final FlywheelSim sim;

    private double integratedPositionRad = 0.0;

    public RollerSim(Config config) {
        super(config.gearing);
        this.sim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        config.motor, config.momentOfInertiaKgM2, config.gearing),
                config.motor);
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }


    @Override
    public double getPosition() {
        return integratedPositionRad;
    }

    @Override
    public double getVelocity() {
        return sim.getAngularVelocityRadPerSec();
    }


    public double getVelocityRPM() {
        return sim.getAngularVelocityRPM();
    }

    @Override
    public double getCurrentAmps() {
        return sim.getCurrentDrawAmps();
    }

    @Override
    public void setState(double positionRad, double velocityRadPerSec) {
        integratedPositionRad = positionRad;
        sim.setAngularVelocity(velocityRadPerSec);
    }

    @Override
    protected void update(double dtSeconds) {
        sim.update(dtSeconds);
        integratedPositionRad += sim.getAngularVelocityRadPerSec() * dtSeconds;
    }


    public static class Config {
        public DCMotor motor = DCMotor.getKrakenX60(1);
        public double gearing = 1.0;


        public double momentOfInertiaKgM2 = 0.004;

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
    }
}
