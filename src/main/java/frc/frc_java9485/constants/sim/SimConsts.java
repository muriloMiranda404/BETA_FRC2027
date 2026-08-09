package frc.frc_java9485.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.frc_java9485.constants.mechanisms.shooter.HoodConsts;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;


public class SimConsts {

    private SimConsts() {}


    public static final class Turret {
        public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
        public static final double GEARING = TurretConsts.Config.TURRET_REDUCTION;


        public static final double MOI_KG_M2 = 0.08;


        public static final double ARM_LENGTH_M = 0.25;

        public static final double MIN_ANGLE_RAD = Math.toRadians(TurretConsts.Setpoint.MIN_TURN_ANGLE_DEG);
        public static final double MAX_ANGLE_RAD = Math.toRadians(TurretConsts.Setpoint.MAX_TURN_ANGLE_DEG);


        public static final double SIM_KP = 0.35;
        public static final double SIM_KD = 0.02;
    }


    public static final class Hood {
        public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
        public static final double GEARING = 40.0;

        public static final double MOI_KG_M2 = 0.012;
        public static final double ARM_LENGTH_M = 0.15;


        public static final double RADIANS_PER_POSITION = Math.toRadians(12.0);

        public static final double MIN_ANGLE_RAD = HoodConsts.Setpoint.MIN_POSITION * RADIANS_PER_POSITION;
        public static final double MAX_ANGLE_RAD = HoodConsts.Setpoint.MAX_POSITION * RADIANS_PER_POSITION;


        public static final double SIM_KP = 6.0;
        public static final double SIM_KD = 0.1;
    }


    public static final class FlyWheel {
        public static final DCMotor MOTOR = DCMotor.getKrakenX60(2);
        public static final double GEARING = 1.0;


        public static final double MOI_KG_M2 = 0.006;


        public static final double SIM_KP = 0.002;
        public static final double SIM_KV = 12.0 / 6000.0;
    }


    public static final class Intake {
        public static final DCMotor PIVOT_MOTOR = DCMotor.getKrakenX60(1);
        public static final double PIVOT_GEARING = 60.0;
        public static final double PIVOT_MOI_KG_M2 = 0.05;
        public static final double PIVOT_ARM_LENGTH_M = 0.35;


        public static final double RADIANS_PER_POSITION = Math.toRadians(0.5);

        public static final double SIM_KP = 0.15;
        public static final double SIM_KD = 0.005;

        public static final DCMotor ROLLER_MOTOR = DCMotor.getKrakenX60(1);
        public static final double ROLLER_GEARING = 3.0;
        public static final double ROLLER_MOI_KG_M2 = 0.002;
    }


    public static final class Index {
        public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
        public static final double GEARING = 5.0;
        public static final double MOI_KG_M2 = 0.002;
    }


    public static final class Conveyor {
        public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
        public static final double GEARING = 30.0;
        public static final double CARRIAGE_MASS_KG = 2.0;
        public static final double DRUM_RADIUS_M = 0.02;


        public static final double TRAVEL_M = 0.5;
    }


    public static final class Climber {
        public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
        public static final double GEARING = 100.0;


        public static final double CARRIAGE_MASS_KG = 55.0;

        public static final double DRUM_RADIUS_M = 0.025;
        public static final double TRAVEL_M = 0.6;
    }
}
