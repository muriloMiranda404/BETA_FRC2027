package frc.robot.subsystems.mechanism;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.mechanism.conveyor.ConveyorIO;
import frc.robot.subsystems.mechanism.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.mechanism.index.IndexIO;
import frc.robot.subsystems.mechanism.index.IndexSubsystem;
import frc.robot.subsystems.mechanism.intake.IntakeIO;
import frc.robot.subsystems.mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanism.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanism.shooter.ShooterSubsystem.ShooterWantedState;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIO;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIO;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIO;

public class SuperStructure extends SubsystemBase{

    private final IntakeSubsystem intake;
    private final IndexSubsystem index;
    private final ConveyorSubsystem conveyor;
    private final ShooterSubsystem shooter;
    private final RobotState robotState;
    private final RobotContainer container;

    private WantedState wantedState = WantedState.OFF;
    private SystemState currentState = SystemState.OFF;

    public SuperStructure(HoodIO io, TurretIO io2, FlyWheelIO io3, IntakeIO io4,
                          IndexIO io5, ConveyorIO io6, Supplier<Pose2d> poseSupplier){
        this.shooter = new ShooterSubsystem(io2, io, io3, poseSupplier);
        this.intake = new IntakeSubsystem(io4);
        this.index = new IndexSubsystem(io5);
        this.conveyor = new ConveyorSubsystem(io6);
        this.container = new RobotContainer();
        this.robotState = container.getRobotState();
    }

    @Override
    public void periodic() {
        this.currentState = handleTransition();
        this.executeActions();
    }

    private void executeActions(){
        switch (currentState) {
            case SHOOTING:
                shooter.setWantedState(ShooterWantedState.AIMING);
                intake.setWantedState(IntakeSubsystem.WantedState.SAVED);
                index.setWantedState(IndexSubsystem.WantedState.INDEXING);
                conveyor.setWantedState(ConveyorSubsystem.WantedState.EXPANDING);
            break;

            case PASSING:
                shooter.setWantedState(ShooterWantedState.PASSING);
                intake.setWantedState(IntakeSubsystem.WantedState.SAVED);
                index.setWantedState(IndexSubsystem.WantedState.INDEXING);
                conveyor.setWantedState(ConveyorSubsystem.WantedState.EXPANDING);
            break;

            case COLLECTING:
                shooter.setWantedState(ShooterWantedState.OFF);
                intake.setWantedState(IntakeSubsystem.WantedState.COLLECTING);
                index.setWantedState(IndexSubsystem.WantedState.STOPPED);
                conveyor.setWantedState(ConveyorSubsystem.WantedState.EXPANDING);
            break;

            case OFF:
                shooter.setWantedState(ShooterWantedState.OFF);
                intake.setWantedState(IntakeSubsystem.WantedState.SAVED);
                index.setWantedState(IndexSubsystem.WantedState.STOPPED);
                conveyor.setWantedState(ConveyorSubsystem.WantedState.STOPPED);
            break;

            case EJECTING_BY_INTAKE:
                shooter.setWantedState(ShooterWantedState.OFF);
                intake.setWantedState(IntakeSubsystem.WantedState.EJECTING);
                index.setWantedState(IndexSubsystem.WantedState.EJECTING);
                conveyor.setWantedState(ConveyorSubsystem.WantedState.STOPPED);
            break;
            default:
                break;
        }
    }

    public void setWantedState(WantedState wantedState){
        this.wantedState = wantedState;
    }

    private SystemState handleTransition(){
        return SystemState.valueOf(wantedState.name());
    }

    public enum WantedState {
        SHOOTING,
        PASSING,
        COLLECTING,
        EJECTING_BY_INTAKE,
        OFF
    }

    private enum SystemState{
        SHOOTING,
        PASSING,
        COLLECTING,
        EJECTING_BY_INTAKE,
        OFF
    }
}
