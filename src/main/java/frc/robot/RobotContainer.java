package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

    private RobotState robotState;

    public RobotContainer(){
        configureBindings();
    }

    public RobotState getRobotState(){
        return robotState;
    }

    private void configureBindings(){}

    public Command getAutonomousCommand(){
        return Commands.none();
    }
}
