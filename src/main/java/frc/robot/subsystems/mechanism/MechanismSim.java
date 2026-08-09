package frc.robot.subsystems.mechanism;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.frc_java9485.constants.mechanisms.shooter.TurretConsts;
import frc.robot.subsystems.mechanism.SuperStructure.SystemState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;


public class MechanismSim {


    private static final int FUEL_CAPACITY = 40;
    private static final double INTAKE_WIDTH_M = 0.6;
    private static final double INTAKE_EXTENSION_M = 0.2;


    private static final double LAUNCH_SPEED_MPS = 12.0;
    private static final double LAUNCH_PITCH_DEG = 60.0;
    private static final int LOOPS_BETWEEN_SHOTS = 10;

    private final SuperStructure superStructure;
    private final SwerveSubsystem swerve;
    private final IntakeSimulation intakeSim;

    private final Translation2d shooterOffsetOnRobot;
    private final double shooterHeightM;

    private int loopsSinceShot = LOOPS_BETWEEN_SHOTS;

    public MechanismSim(SuperStructure superStructure, SwerveSubsystem swerve) {
        this.superStructure = superStructure;
        this.swerve = swerve;

        this.intakeSim = IntakeSimulation.OverTheBumperIntake(
            "Fuel", swerve.getSimulation(),
            Meters.of(INTAKE_WIDTH_M), Meters.of(INTAKE_EXTENSION_M),
            IntakeSide.FRONT, FUEL_CAPACITY);
        this.intakeSim.register();

        var turretTransform = TurretConsts.Config.ROBOT_TO_TURRET_TRANSFORM;
        this.shooterOffsetOnRobot = new Translation2d(turretTransform.getX(), turretTransform.getY());
        this.shooterHeightM = turretTransform.getZ();
    }


    public void update() {
        SystemState state = superStructure.getCurrentState();

        if (state == SystemState.COLLECTING) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }

        boolean feeding = state == SystemState.SHOOTING && superStructure.isReadyToShoot();
        if (feeding && intakeSim.getGamePiecesAmount() > 0) {
            loopsSinceShot++;
            if (loopsSinceShot >= LOOPS_BETWEEN_SHOTS && intakeSim.obtainGamePieceFromIntake()) {
                loopsSinceShot = 0;
                launchFuel();
            }
        } else {

            loopsSinceShot = LOOPS_BETWEEN_SHOTS;
        }

        Logger.recordOutput("MechanismSim/HeldFuel", intakeSim.getGamePiecesAmount());
    }

    private void launchFuel() {
        Translation2d robotPos = swerve.getSimulation().getSimulatedDriveTrainPose().getTranslation();
        Translation3d target = superStructure.getCurrentTarget();

        Rotation2d shooterFacing = new Rotation2d(
            target.getX() - robotPos.getX(), target.getY() - robotPos.getY());

        RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(
            robotPos,
            shooterOffsetOnRobot,
            swerve.getSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            shooterFacing,
            Meters.of(shooterHeightM),
            MetersPerSecond.of(LAUNCH_SPEED_MPS),
            Degrees.of(LAUNCH_PITCH_DEG));

        SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    }
}
