package frc.robot.subsystems.swerve;

public class StaticSwerve {

    private static final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

    public StaticSwerve(){}

    public static double getMaxAngularVelocity(){
        return swerveSubsystem.getMaxAngularVelocity();
    }


    public static double getMeasuredAngularVelocity(){
        return swerveSubsystem.getRobotRelativeSpeeds().omegaRadiansPerSecond;
    }

    public static double getYaw(){
        return swerveSubsystem.getYaw();
    }

    public static double getPitch(){
        return swerveSubsystem.getPitch();
    }

    public static double getRoll(){
        return swerveSubsystem.getRoll();
    }

    public static void driveToDriverStation(){
        swerveSubsystem.driveToNearestCoralStation();
    }
}
