package frc.frc_java9485.utils.geometry;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;


public record PoseErrorTolerance(Distance linearErrorTolerance, Angle angularErrorTolerance) {


    public PoseErrorTolerance(double linearErrorToleranceMeters, Rotation2d angularErrorTolerance) {
        this(Meters.of(linearErrorToleranceMeters), angularErrorTolerance.getMeasure());
    }


    public static PoseErrorTolerance of(double linearMeters, double angularDegrees) {
        return new PoseErrorTolerance(Meters.of(linearMeters), Degrees.of(angularDegrees));
    }


    public boolean atPose(Pose2d expected, Pose2d actual) {
        return atTranslation(expected, actual) && atRotation(expected, actual);
    }


    public boolean atTranslation(Pose2d expected, Pose2d actual) {
        double linearError = expected.getTranslation().getDistance(actual.getTranslation());
        return MathUtil.isNear(0.0, linearError, linearErrorTolerance.in(Meters));
    }


    public boolean atRotation(Pose2d expected, Pose2d actual) {
        return MathUtil.isNear(
                expected.getRotation().getDegrees(),
                actual.getRotation().getDegrees(),
                angularErrorTolerance.in(Degrees),
                -180.0,
                180.0);
    }
}
