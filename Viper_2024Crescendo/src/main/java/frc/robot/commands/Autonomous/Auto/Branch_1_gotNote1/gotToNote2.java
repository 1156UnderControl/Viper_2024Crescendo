package frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.AutonomousAimSwerveSpeaker;
import frc.robot.commands.Autonomous.AutonomousGoToNote;
import frc.robot.commands.Autonomous.AutonomousShootInterpolation;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillDynamic;

public class gotToNote2 extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 3 LEFT");

  public gotToNote2(RobotSubsystems robot) {
    addCommands(
        new AutonomousGoToNote(robot),
        new InstantCommand(() -> AutoStealOverkillDynamic.gotNotes[1] = true),
        AutoBuilder.followPath(path1),
        new AutonomousAimSwerveSpeaker(robot),
        new AutonomousShootInterpolation(robot));
  }
}
