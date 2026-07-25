package frc.robot.commands.swerveUtils.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.constants.utils.FieldElementsConst;
import frc.robot.subsystems.mechanism.shooter.turret.TurretCalculator;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Gira o robô no lugar para que sua frente aponte para o HUB da aliança atual,
 * deixando-o alinhado para pontuar.
 *
 * Reaproveita:
 *  - {@link TurretCalculator#calculateTurretAngle} para descobrir quanto o robô
 *    precisa girar (relativo ao robô) até apontar para o alvo.
 *  - {@link FieldElementsConst.HubMeansured} para a posição do HUB (RED/BLUE).
 *  - O mesmo padrão de {@link TurnRobotToAngle}: um {@link PIDController} de
 *    heading acionando {@link SwerveSubsystem#drive}.
 */
public class AimRobotToHub extends Command {

    private final SwerveSubsystem swerve;
    private final PIDController controller;

    private Translation3d hubTarget;

    public AimRobotToHub(SwerveSubsystem swerve) {
        this.swerve = swerve;

        this.controller = new PIDController(0.01, 0, 0);
        this.controller.setTolerance(2);
        this.controller.enableContinuousInput(-180.0, 180.0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        controller.reset();

        this.hubTarget = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red
                ? FieldElementsConst.HubMeansured.HUB_RED
                : FieldElementsConst.HubMeansured.HUB_BLUE)
            .orElse(FieldElementsConst.HubMeansured.HUB_BLUE);
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose2d();

        double headingDeg = robotPose.getRotation().getDegrees();
        double targetHeadingDeg = headingDeg
            + TurretCalculator.calculateTurretAngle(robotPose, hubTarget);

        controller.setSetpoint(targetHeadingDeg);

        double output = controller.calculate(headingDeg);
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
