package frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1RIGHT;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillRIGHTDynamic;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSwerveSpeaker;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousShootInterpolation;

public class gotNote1 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 2 LEFT RED");

  public gotNote1(RobotSubsystems robot) {
    addCommands(
        new InstantCommand(() -> AutoStealOverkillRIGHTDynamic.gotNotes[1] = true),
        new InstantCommand(() -> Robot.autonomousState = "GOT_SECOND_NOTE"),
        new AutonomousAimSwerveSpeaker(robot),
        new AutonomousShootInterpolation(robot));
  }
}
