package frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1LEFT;

import static edu.wpi.first.wpilibj2.command.Commands.either;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillLEFTDynamic;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimAtAngle;

public class GoToNote3 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 3 dynamic left");
  private RobotSubsystems robot;

  public GoToNote3(RobotSubsystems robot) {
    this.robot = robot;
    addCommands(
        new InstantCommand(() -> AutoStealOverkillLEFTDynamic.gotNotes[2] = true),
        either(new AutonomousAimAtAngle(robot, 90), new AutonomousAimAtAngle(robot, 180),
            () -> AutoStealOverkillLEFTDynamic.gotNotes[1]));
  }
}
