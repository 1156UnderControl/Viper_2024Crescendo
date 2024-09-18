package frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1RIGHT;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillRIGHTDynamic;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimAtAngle;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSwerveSpeaker;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousShootInterpolation;

public class gotNote2 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 3 LEFT");

  public gotNote2(RobotSubsystems robot) {
    addCommands(
        new InstantCommand(() -> AutoStealOverkillRIGHTDynamic.gotNotes[0] = true),
        new AutonomousAimSwerveSpeaker(robot),
        new AutonomousShootInterpolation(robot),
        new AutonomousAimAtAngle(robot, 90));
  }
}
