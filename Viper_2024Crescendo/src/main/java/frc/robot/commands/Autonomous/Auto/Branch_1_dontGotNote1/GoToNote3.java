package frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.AutonomousGoToNote;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillDynamic;

public class GoToNote3 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 3 dynamic");
  private RobotSubsystems robot;

  public GoToNote3(RobotSubsystems robot) {
    this.robot = robot;
    addCommands(
        new InstantCommand(() -> AutoStealOverkillDynamic.gotNotes[2] = true),
        AutoBuilder.followPath(this.path1),
        new AutonomousGoToNote(robot));
  }
}
