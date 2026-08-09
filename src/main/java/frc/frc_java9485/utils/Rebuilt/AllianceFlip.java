package frc.frc_java9485.utils.Rebuilt;

import static frc.frc_java9485.constants.utils.FieldElementsConst.FieldMeansureds.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;


public class AllianceFlip {
  private AllianceFlip() {}

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }



  public static double flipX(double x) {
    return FIELD_LENGTH_METERS - x;
  }

  public static Pose2d flipPose2dToRed(Pose2d bluePose) {
    return new Pose2d(
        flipX(bluePose.getX()),
        bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose2d flipPose2dToRedAndNormalize(Pose2d bluePose) {
    return new Pose2d(
        flipX(bluePose.getX()),
        bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg).minus(Rotation2d.fromDegrees(360)));
  }

  public static Translation2d flipTranslation2dToRed(Translation2d blueTranslation) {
    return new Translation2d(flipX(blueTranslation.getX()), blueTranslation.getY());
  }

  public static Translation3d flipTranslation3dToRed(Translation3d blueTranslation) {
    return new Translation3d(
        flipX(blueTranslation.getX()), blueTranslation.getY(), blueTranslation.getZ());
  }

  public static Rotation2d flipRotation2dToRed(Rotation2d blueRotation) {
    return blueRotation.plus(Rotation2d.k180deg);
  }

  public static Rotation3d flipRotation3dToRed(Rotation3d blueRotation) {
    return blueRotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI));
  }

  public static Pose3d flipPose3dToRed(Pose3d bluePose) {
    return new Pose3d(
        flipTranslation3dToRed(bluePose.getTranslation()), flipRotation3dToRed(bluePose.getRotation()));
  }




  public static double applyX(double x) {
    return shouldFlip() ? flipX(x) : x;
  }

  public static Translation2d apply(Translation2d blueTranslation) {
    return shouldFlip() ? flipTranslation2dToRed(blueTranslation) : blueTranslation;
  }

  public static Translation3d apply(Translation3d blueTranslation) {
    return shouldFlip() ? flipTranslation3dToRed(blueTranslation) : blueTranslation;
  }

  public static Rotation2d apply(Rotation2d blueRotation) {
    return shouldFlip() ? flipRotation2dToRed(blueRotation) : blueRotation;
  }

  public static Rotation3d apply(Rotation3d blueRotation) {
    return shouldFlip() ? flipRotation3dToRed(blueRotation) : blueRotation;
  }

  public static Pose2d apply(Pose2d bluePose) {
    return shouldFlip() ? flipPose2dToRed(bluePose) : bluePose;
  }

  public static Pose3d apply(Pose3d bluePose) {
    return shouldFlip() ? flipPose3dToRed(bluePose) : bluePose;
  }
}
