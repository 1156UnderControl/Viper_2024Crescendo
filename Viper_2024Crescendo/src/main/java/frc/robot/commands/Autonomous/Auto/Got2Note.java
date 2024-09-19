package frc.robot.commands.Autonomous.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;

public class Got2Note extends SequentialCommandGroup {
  private PathPlannerPath path1Note = PathPlannerPath.fromPathFile("Auto Steal OVK 4N 2");
  private PathPlannerPath path3Note = PathPlannerPath.fromPathFile("Auto Steal OVK 4N 4");
  private RobotSubsystems robot;

  public Got2Note(RobotSubsystems robot) {
    this.robot = robot;
    addCommands(
        AutoBuilder.followPath(this.path1Note),
        AutoBuilder.followPath(this.path3Note));
  }
}
