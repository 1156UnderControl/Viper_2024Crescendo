package frc.robot.commands.Autonomous.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.robot.RobotSubsystems;

public class AutoTestComand extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("no note 1");
  private SendableChooser<Command> autoChooser;
  private RobotSubsystems robot;

  public AutoTestComand(RobotSubsystems robot, SendableChooser<Command> autoChooser) {
    this.robot = robot;
    this.autoChooser = autoChooser;
    addCommands(
        this.autoChooser.getSelected(),
        Commands.either(new frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1.AutoTestComand(robot, autoChooser),
            new frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1.AutoTestComand(robot, autoChooser),
            () -> LimelightHelpers.getTV("limelight-note")));
  }
}
