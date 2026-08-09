package frc.robot.subsystems.mechanism;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.bases.StateGraph;
import frc.frc_java9485.constants.utils.LoggerConstants;
import frc.robot.subsystems.mechanism.conveyor.ConveyorIO;
import frc.robot.subsystems.mechanism.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.mechanism.index.IndexIO;
import frc.robot.subsystems.mechanism.index.IndexSubsystem;
import frc.robot.subsystems.mechanism.intake.IntakeIO;
import frc.robot.subsystems.mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanism.shooter.ShooterSubsystem;
import frc.robot.subsystems.mechanism.shooter.ShooterSubsystem.ShooterWantedState;
import frc.robot.subsystems.mechanism.shooter.ShotVerifier;
import frc.robot.subsystems.mechanism.shooter.flyWheel.FlyWheelIO;
import frc.robot.subsystems.mechanism.shooter.hood.HoodIO;
import frc.robot.subsystems.mechanism.shooter.turret.TurretIO;

public class SuperStructure extends SubsystemBase{

    private final IntakeSubsystem intake;
    private final IndexSubsystem index;
    private final ConveyorSubsystem conveyor;
    private final ShooterSubsystem shooter;

    private static final String LOG_KEY = LoggerConstants.MECHANISM_KEY + "SuperStructure/";

    private final StateGraph<SystemState> stateGraph = buildStateGraph();

    @AutoLogOutput
    private WantedState wantedState = WantedState.OFF;

    @AutoLogOutput
    private SystemState currentState = SystemState.OFF;

    public SuperStructure(HoodIO hoodIO, TurretIO turretIO, FlyWheelIO flyWheelIO, IntakeIO intakeIO,
                          IndexIO indexIO, ConveyorIO conveyorIO){
        this.shooter = new ShooterSubsystem(turretIO, hoodIO, flyWheelIO);
        this.intake = new IntakeSubsystem(intakeIO);
        this.index = new IndexSubsystem(indexIO);
        this.conveyor = new ConveyorSubsystem(conveyorIO);
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
            this.wantedState = WantedState.OFF;
        }

        this.currentState = handleTransition();
        this.executeActions();


        intake.update();
        index.update();
        conveyor.update();
    }

    private void executeActions(){
        switch (currentState) {
            case SHOOTING:
                shooter.setWantedState(ShooterWantedState.AIMING);
                intake.setWantedState(IntakeSubsystem.WantedState.SAVED);


                boolean readyToShoot = shooter.isReadyToShoot();
                index.setWantedState(readyToShoot
                        ? IndexSubsystem.WantedState.INDEXING
                        : IndexSubsystem.WantedState.STOPPED);
                conveyor.setWantedState(readyToShoot
                        ? ConveyorSubsystem.WantedState.EXPANDING
                        : ConveyorSubsystem.WantedState.STOPPED);
            break;

            case PREPARING:

                shooter.setWantedState(ShooterWantedState.AIMING);
                intake.setWantedState(IntakeSubsystem.WantedState.SAVED);
                index.setWantedState(IndexSubsystem.WantedState.STOPPED);
                conveyor.setWantedState(ConveyorSubsystem.WantedState.STOPPED);
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

    public WantedState getWantedState(){
        return this.wantedState;
    }

    public SystemState getCurrentState(){
        return this.currentState;
    }

    public boolean isInState(SystemState state){
        return this.currentState == state;
    }


    public boolean isReadyToShoot(){
        return shooter.isReadyToShoot();
    }


    public ShotVerifier.Rejection getShotRejection(){
        return shooter.getShotRejection();
    }


    public boolean mechanismsAtSetpoint(){
        return shooter.mechanismsAtSetpoint();
    }


    public Command prepare(){
        return startEnd(() -> setWantedState(WantedState.PREPARING), () -> setWantedState(WantedState.OFF));
    }


    public double getTurretAngleDeg(){
        return shooter.getTurretAngleDeg();
    }


    public double getHoodPosition(){
        return shooter.getHoodPosition();
    }


    public double getIntakePivotPosition(){
        return intake.getPivotPosition();
    }


    public Translation3d getCurrentTarget(){
        return shooter.getCurrentTarget();
    }



    public Command shoot(){
        return startEnd(() -> setWantedState(WantedState.SHOOTING), () -> setWantedState(WantedState.OFF));
    }

    public Command collect(){
        return startEnd(() -> setWantedState(WantedState.COLLECTING), () -> setWantedState(WantedState.OFF));
    }

    public Command pass(){
        return startEnd(() -> setWantedState(WantedState.PASSING), () -> setWantedState(WantedState.OFF));
    }

    public Command eject(){
        return startEnd(() -> setWantedState(WantedState.EJECTING_BY_INTAKE), () -> setWantedState(WantedState.OFF));
    }


    private static StateGraph<SystemState> buildStateGraph() {
        StateGraph<SystemState> graph = new StateGraph<>(SystemState.class);

        graph.connectAllThrough(SystemState.OFF, 0.25);
        graph.addBidirectional(SystemState.SHOOTING, SystemState.PASSING, 0.15);

        graph.addBidirectional(SystemState.SHOOTING, SystemState.PREPARING, 0.05);
        graph.addBidirectional(SystemState.PASSING, SystemState.PREPARING, 0.15);

        return graph;
    }

    private SystemState handleTransition(){
        SystemState goal = switch (wantedState) {
            case SHOOTING -> SystemState.SHOOTING;
            case PASSING -> SystemState.PASSING;
            case PREPARING -> SystemState.PREPARING;
            case COLLECTING -> SystemState.COLLECTING;
            case EJECTING_BY_INTAKE -> SystemState.EJECTING_BY_INTAKE;
            case OFF -> SystemState.OFF;
        };


        SystemState next = stateGraph.nextStepToward(currentState, goal);

        Logger.recordOutput(LOG_KEY + "GoalState", goal.toString());
        Logger.recordOutput(LOG_KEY + "AtGoal", next == goal);

        return next;
    }

    public enum WantedState {
        SHOOTING,
        PASSING,

        PREPARING,
        COLLECTING,
        EJECTING_BY_INTAKE,
        OFF
    }

    public enum SystemState{
        SHOOTING,
        PASSING,
        PREPARING,
        COLLECTING,
        EJECTING_BY_INTAKE,
        OFF
    }
}
