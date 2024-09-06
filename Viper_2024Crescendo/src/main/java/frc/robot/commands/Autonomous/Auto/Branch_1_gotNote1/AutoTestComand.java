package frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.AutonomousAimSubwoofer;
import frc.robot.commands.Autonomous.AutonomousGoToNote;
import frc.robot.commands.Autonomous.AutonomousShoot;

public class AutoTestComand extends SequentialCommandGroup {
  private PathPlannerPath path = PathPlannerPath.fromPathFile("yes note 1");
  private RobotSubsystems robot;

  public AutoTestComand(RobotSubsystems robot, SendableChooser<Command> autoChooser) {
    this.robot = robot;
    addCommands(
        new AutonomousGoToNote(this.robot),
        AutoBuilder.followPath(this.path),
        new AutonomousAimSubwoofer(robot),
        new AutonomousShoot(robot));
  }
}
