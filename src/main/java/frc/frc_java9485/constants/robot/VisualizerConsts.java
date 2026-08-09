package frc.frc_java9485.constants.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


public class VisualizerConsts {

    private VisualizerConsts() {}


    public static final int TURRET_INDEX = 0;
    public static final int HOOD_INDEX = 1;
    public static final int INTAKE_INDEX = 2;
    public static final int CLIMBER_INDEX = 3;
    public static final int COMPONENT_COUNT = 4;


    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(
            new Translation3d(Inches.zero(), Inches.of(7), Inches.of(17.5)), Rotation3d.kZero);


    public static final Transform3d TURRET_TO_HOOD = new Transform3d(
            new Translation3d(Inches.of(4), Inches.zero(), Inches.of(6)), Rotation3d.kZero);


    public static final Transform3d ROBOT_TO_INTAKE_PIVOT = new Transform3d(
            new Translation3d(Inches.of(13), Inches.zero(), Inches.of(8)), Rotation3d.kZero);


    public static final Transform3d ROBOT_TO_CLIMBER = new Transform3d(
            new Translation3d(Inches.of(-10), Inches.zero(), Inches.of(6)), Rotation3d.kZero);


    public static final double HOOD_DEGREES_PER_POSITION = 12.0;


    public static final double CLIMBER_METERS_PER_POSITION = 0.02;
}
