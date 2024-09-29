package frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1LEFT;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillLEFTDynamic;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimAtAngle;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousGoToNote;

public class goToNote2 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 3 LEFT");
  private PathPlannerPath path1IfCatchedNote2 = PathPlannerPath.fromPathFile("steal ovk path 2 LEFT RED");

  public goToNote2(RobotSubsystems robot) {
    addCommands(
        either(AutoBuilder.followPath(path1IfCatchedNote2), none(), () -> AutoStealOverkillLEFTDynamic.gotNotes[1]),
        new AutonomousAimAtAngle(robot, 0),
        new AutonomousGoToNote(robot),
        AutoBuilder.followPath(path1),
        either(new gotNote2(robot), none(), () -> robot.robotHasNote()),
        new AutonomousAimAtAngle(robot, 90));
  }
}
