package frc.robot.subsystems.swerve;

public class StaticSwerve {
    
    private static final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    
    public StaticSwerve(){}

    public static double getAngularVelocity(){
        return swerveSubsystem.getAngularVelocity();
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
}
