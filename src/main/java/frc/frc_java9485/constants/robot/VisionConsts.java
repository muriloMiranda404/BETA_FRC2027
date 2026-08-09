package frc.frc_java9485.constants.robot;

import static frc.frc_java9485.constants.robot.RobotConsts.isSimulation;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConsts {

    public static final boolean RASPBERRY_ENABLED = false;

    public static final String RASPBERRY_CAMERA_NAME = "raspphoto";


    public static final Transform3d RASPBERRY_ROBOT_TO_CAMERA =
        new Transform3d(new Translation3d(), new Rotation3d());


    public static final String LIMELIGHT_CAMERA_NAME = "limelight-hyobots";

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);


    public static SimCameraProperties RASPBERRY_CAMERA_PROPS;

    static {
        if (isSimulation()) {
            RASPBERRY_CAMERA_PROPS = new SimCameraProperties();
            RASPBERRY_CAMERA_PROPS.setFPS(50);
            RASPBERRY_CAMERA_PROPS.setAvgLatencyMs(35);
            RASPBERRY_CAMERA_PROPS.setLatencyStdDevMs(5);
            RASPBERRY_CAMERA_PROPS.setCalibError(0.5, 0.5);
            RASPBERRY_CAMERA_PROPS.setCalibration(640, 480, Rotation2d.fromDegrees(110));
        }
    }
}
