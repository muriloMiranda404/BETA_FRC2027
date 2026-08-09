package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.frc_java9485.constants.robot.RobotConsts;
import frc.frc_java9485.motors.ctre.phoenix6.StatusSignalRefresher;
import frc.frc_java9485.utils.Rebuilt.HubTracker;
import frc.frc_java9485.utils.Rebuilt.Simulation;
import frc.frc_java9485.utils.Rebuilt.Zones;
import frc.frc_java9485.utils.VirtualSubsystem;
import frc.frc_java9485.utils.logger.Elastic;
import frc.frc_java9485.utils.logger.LoggedTracer;
import frc.frc_java9485.utils.logger.VirtualPD;
import frc.frc_java9485.utils.logger.wpilogxz.WPILOGXZWriter;
import frc.frc_java9485.utils.logger.Elastic.Notification;
import frc.frc_java9485.utils.logger.Elastic.Notification.NotificationLevel;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static edu.wpi.first.units.Units.Seconds;
import static frc.frc_java9485.constants.utils.FieldElementsConst.SimulationPoses.*;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private double currentMatchTime;
  private boolean runnedAutonomous;
  private boolean hubWasActive;

  private final Timer timer;
  private final PowerDistribution powerDistribution;
  private final RobotAlerts robotAlerts;

  private final Optional<Time> shiftTime = HubTracker.timeRemainingInCurrentShift();

  private Simulation simulator;
  private final SwerveSubsystem swerve;
  private final RobotContainer m_robotContainer;

  public Robot() {
    switch (RobotConsts.CURRENT_ROBOT_MODE) {
      case REAL:
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(RobotConsts.USE_COMPRESSED_LOGS
            ? new WPILOGXZWriter(RobotConsts.LOGS_PATH)
            : new WPILOGWriter(RobotConsts.LOGS_PATH));
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        simulator = Simulation.getInstance();
        break;
    }


    Logger.start();

    m_robotContainer = new RobotContainer();
    swerve = SwerveSubsystem.getInstance();


    StatusSignalRefresher.getInstance().finalizeStatusSignals();

    timer = new Timer();
    powerDistribution = new PowerDistribution();
    robotAlerts = new RobotAlerts(powerDistribution);

    currentMatchTime = 0.00;
    runnedAutonomous = false;

    if(shiftTime.isPresent()){
      SmartDashboard.putNumber("time do shift", shiftTime.get().in(Seconds));
    }
  }

  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());


    Zones.logAllZones();
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();


    StatusSignalRefresher.getInstance().refreshStatusSignals();
    LoggedTracer.record("StatusSignals");


    VirtualSubsystem.runAllPeriodic();
    LoggedTracer.record("VirtualSubsystems");

    CommandScheduler.getInstance().run();
    LoggedTracer.record("CommandScheduler");

    VirtualSubsystem.runAllPeriodicAfterScheduler();
    LoggedTracer.record("VirtualSubsystemsAfter");

    String gameMessage = DriverStation.getGameSpecificMessage();
    RobotState.getInstance().setGameSpecificMessage(gameMessage.isEmpty() ? null : gameMessage);

    robotAlerts.update();
    VirtualPD.logAll();

    SmartDashboard.putBoolean("hub is active", HubTracker.isActive());
    SmartDashboard.putNumber("batery voltage", powerDistribution.getVoltage());

    if (powerDistribution.getVoltage() <= 11.4
        && timer.advanceIfElapsed(10)
        && !DriverStation.isFMSAttached()) {
      String desc = String.format("Bateria com %.2f Volts", powerDistribution.getVoltage());
      Elastic.sendNotification(
          new Notification(NotificationLevel.WARNING, "BATERIA BAIXA!!", desc));
    }

    currentMatchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("Match Time", currentMatchTime);

    LoggedTracer.record("RobotPeriodic");
  }

  @Override
  public void autonomousInit() {
    Elastic.sendNotification(
        new Notification(NotificationLevel.INFO, "Inicio do Autonomous!!", ""));

    runnedAutonomous = true;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Inicio do teleop!!", ""));
      if (!runnedAutonomous) {
        var alliancePosition = DriverStation.getRawAllianceStation();
        switch (alliancePosition) {
          case Blue1:
          System.out.println("resetando...");
            swerve.resetOdometry(BLUE_LEFT_START_POSE);
            break;
          case Blue2:
            swerve.resetOdometry(BLUE_CENTER_START_POSE);
            break;
          case Blue3:
            swerve.resetOdometry(BLUE_RIGHT_START_POSE);
            break;

          case Red1:
            swerve.resetOdometry(RED_LEFT_START_POSE);
            break;
          case Red2:
            swerve.resetOdometry(RED_CENTER_START_POSE);
            break;
          case Red3:
            swerve.resetOdometry(RED_RIGHT_START_POSE);
            break;

          case Unknown:
            swerve.resetOdometry(FIELD_CENTER_POSE);
            break;
        }
      }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    boolean hubActive = HubTracker.isActive();
    if (hubActive && !hubWasActive) {
      Elastic.sendNotification(new Notification(NotificationLevel.INFO,
          "Hub is active", "the hub is active"));
    }
    hubWasActive = hubActive;
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    simulator.updateArena();
    m_robotContainer.updateSimulation();
  }
}
