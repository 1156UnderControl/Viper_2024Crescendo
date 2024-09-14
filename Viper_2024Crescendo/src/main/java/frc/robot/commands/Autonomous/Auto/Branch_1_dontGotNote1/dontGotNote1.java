package frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillDynamic;

public class dontGotNote1 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 2 dynamic");
  private PathPlannerPath path2 = PathPlannerPath.fromPathFile("steal ovk path 3 LEFT");
  private RobotSubsystems robot;

  public dontGotNote1(RobotSubsystems robot) {
    this.robot = robot;
    addCommands(
        new InstantCommand(() -> AutoStealOverkillDynamic.gotNotes[1] = false),
        AutoBuilder.followPath(this.path1));
  }
}
