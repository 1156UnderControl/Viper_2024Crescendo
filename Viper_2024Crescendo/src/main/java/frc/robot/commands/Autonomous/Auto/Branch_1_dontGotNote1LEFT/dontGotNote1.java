package frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1LEFT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillLEFTDynamic;

public class dontGotNote1 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 2 dynamic left");
  private PathPlannerPath path2 = PathPlannerPath.fromPathFile("steal ovk path 3 LEFT");
  private RobotSubsystems robot;

  public dontGotNote1(RobotSubsystems robot) {
    this.robot = robot;
    addCommands(
        new InstantCommand(() -> AutoStealOverkillLEFTDynamic.gotNotes[1] = false),
        new InstantCommand(() -> Robot.autonomousState = "DIDNT_GOT_SECOND_NOTE"),
        AutoBuilder.followPath(this.path1));
  }
}
