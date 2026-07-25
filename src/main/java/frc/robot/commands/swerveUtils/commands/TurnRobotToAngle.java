package frc.robot.commands.swerveUtils.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TurnRobotToAngle extends Command {

    private final SwerveSubsystem swerve;
    private final double targetDeg;
    private final PIDController controller;

    public TurnRobotToAngle(SwerveSubsystem swerve, double targetDeg) {
        this.swerve = swerve;
        this.targetDeg = targetDeg;

        this.controller = new PIDController(0.01,
                                            0,
                                            0);

        this.controller.setTolerance(1);
        this.controller.enableContinuousInput(-180.0, 180.0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        controller.reset();
        controller.setSetpoint(targetDeg);
    }

    @Override
    public void execute() {
        double output = controller.calculate(swerve.getYaw());
        swerve.drive(new Translation2d(0, 0), output, true);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}
