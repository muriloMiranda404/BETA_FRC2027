
package frc.robot.subsystems.mechanism;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.frc_java9485.constants.robot.VisualizerConsts;
import frc.frc_java9485.utils.VirtualSubsystem;


public class MechanismVisualizer extends VirtualSubsystem {

    private static final String LOG_KEY = "Visualizer/";

    private final DoubleSupplier turretAngleDeg;
    private final DoubleSupplier hoodPosition;
    private final DoubleSupplier intakePivotPosition;
    private final DoubleSupplier climberPosition;
    private final java.util.function.Supplier<Pose3d> robotPose;


    public MechanismVisualizer(
            DoubleSupplier turretAngleDeg,
            DoubleSupplier hoodPosition,
            DoubleSupplier intakePivotPosition,
            DoubleSupplier climberPosition,
            java.util.function.Supplier<Pose3d> robotPose) {
        this.turretAngleDeg = turretAngleDeg;
        this.hoodPosition = hoodPosition;
        this.intakePivotPosition = intakePivotPosition;
        this.climberPosition = climberPosition;
        this.robotPose = robotPose;
    }

    @Override
    public void periodic() {
        Pose3d[] components = new Pose3d[VisualizerConsts.COMPONENT_COUNT];


        Pose3d turret = Pose3d.kZero
                .transformBy(VisualizerConsts.ROBOT_TO_TURRET)
                .transformBy(new Transform3d(
                        Translation3d.kZero,
                        new Rotation3d(0.0, 0.0, Math.toRadians(turretAngleDeg.getAsDouble()))));
        components[VisualizerConsts.TURRET_INDEX] = turret;


        double hoodPitchRad = Math.toRadians(
                hoodPosition.getAsDouble() * VisualizerConsts.HOOD_DEGREES_PER_POSITION);
        components[VisualizerConsts.HOOD_INDEX] = turret
                .transformBy(VisualizerConsts.TURRET_TO_HOOD)
                .transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -hoodPitchRad, 0.0)));


        components[VisualizerConsts.INTAKE_INDEX] = Pose3d.kZero
                .transformBy(VisualizerConsts.ROBOT_TO_INTAKE_PIVOT)
                .transformBy(new Transform3d(
                        Translation3d.kZero,
                        new Rotation3d(0.0, Math.toRadians(intakePivotPosition.getAsDouble()), 0.0)));


        double climberRise = climberPosition.getAsDouble() * VisualizerConsts.CLIMBER_METERS_PER_POSITION;
        components[VisualizerConsts.CLIMBER_INDEX] = Pose3d.kZero
                .transformBy(VisualizerConsts.ROBOT_TO_CLIMBER)
                .transformBy(new Transform3d(new Translation3d(0.0, 0.0, climberRise), Rotation3d.kZero));

        Logger.recordOutput(LOG_KEY + "Components", components);

        Pose3d fieldToRobot = robotPose.get();
        Pose3d[] fieldComponents = new Pose3d[components.length];
        for (int i = 0; i < components.length; i++) {
            fieldComponents[i] = fieldToRobot.transformBy(
                    new Transform3d(components[i].getTranslation(), components[i].getRotation()));
        }
        Logger.recordOutput(LOG_KEY + "ComponentsField", fieldComponents);
    }

    @Override
    public void periodicAfterScheduler() {

    }
}
