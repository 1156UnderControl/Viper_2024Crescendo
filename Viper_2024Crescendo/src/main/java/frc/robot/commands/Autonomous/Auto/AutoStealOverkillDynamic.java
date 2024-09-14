
package frc.robot.commands.Autonomous.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.either;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1.GoToNote3;
import frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1.dontGotNote1;
import frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1.gotNote1;
import frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1.gotToNote2;

public class AutoStealOverkillDynamic extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("no note 1");
  private SendableChooser<Command> autoChooser;
  private RobotSubsystems robot;
  private Timer timer = new Timer();

  public static boolean gotNotes[] = { false, false, false };

  public AutoStealOverkillDynamic(RobotSubsystems robot, SendableChooser<Command> autoChooser) {
    this.robot = robot;
    this.autoChooser = autoChooser;
    autoChooser.getSelected().getName();
    addCommands(
        new InstantCommand(() -> timer.start()),
        this.autoChooser.getSelected().until(() -> timer.get() >= 5.2 && !robot.shooter.hasNote()),
        either(new dontGotNote1(robot), new gotNote1(robot), () -> !robot.shooter.hasNote()),
        either(new gotToNote2(robot), new GoToNote3(robot), () -> LimelightHelpers.getTV("limelight-note")));
  }

}
