package frc.robot.commands.Autonomous.Auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Autonomous.AutonomousGoToNote;

public class AutoTestComand extends SequentialCommandGroup {
  ControlBoard mControlBoard = ControlBoard.getInstance();

  public AutoTestComand(RobotSubsystems robot) {
    addCommands(
        AutoBuilder.followPath(null),
        new AutonomousGoToNote(robot));
  }
}
