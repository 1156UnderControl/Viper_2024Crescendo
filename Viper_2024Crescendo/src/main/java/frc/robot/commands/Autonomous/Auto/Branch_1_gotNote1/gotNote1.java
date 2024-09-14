package frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.AutonomousAimSwerveSpeaker;
import frc.robot.commands.Autonomous.AutonomousShootInterpolation;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillDynamic;

public class gotNote1 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 2 LEFT RED");

  public gotNote1(RobotSubsystems robot) {
    addCommands(
        new InstantCommand(() -> AutoStealOverkillDynamic.gotNotes[1] = true),
        new AutonomousAimSwerveSpeaker(robot),
        new AutonomousShootInterpolation(robot),
        AutoBuilder.followPath(path1));
  }
}
