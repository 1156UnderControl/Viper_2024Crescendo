package frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillDynamic;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimAtAngle;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSwerveSpeaker;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousGoToNote;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousShootInterpolation;

public class goToNote2 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 3 LEFT");
  private PathPlannerPath path1IfCatchedNote2 = PathPlannerPath.fromPathFile("steal ovk path 2 LEFT RED");

  public goToNote2(RobotSubsystems robot) {
    addCommands(
        either(AutoBuilder.followPath(path1IfCatchedNote2), none(), () -> AutoStealOverkillDynamic.gotNotes[1]),
        new AutonomousGoToNote(robot),
        new InstantCommand(() -> AutoStealOverkillDynamic.gotNotes[0] = true),
        AutoBuilder.followPath(path1),
        new AutonomousAimSwerveSpeaker(robot),
        new AutonomousShootInterpolation(robot),
        new AutonomousAimAtAngle(robot, 90),
        new AutonomousGoToNote(robot));
  }
}
