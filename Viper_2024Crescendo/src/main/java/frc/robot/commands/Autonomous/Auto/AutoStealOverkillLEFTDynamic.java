
package frc.robot.commands.Autonomous.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.either;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1LEFT.GoToNote3;
import frc.robot.commands.Autonomous.Auto.Branch_1_dontGotNote1LEFT.dontGotNote1;
import frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1LEFT.goToNote2;
import frc.robot.commands.Autonomous.Auto.Branch_1_gotNote1LEFT.gotNote1;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSwerveSpeaker;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousCollect;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousGoToNote;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousShootInterpolation;

public class AutoStealOverkillLEFTDynamic extends SequentialCommandGroup {
  private PathPlannerPath path1 = PathPlannerPath.fromPathFile("steal ovk path 5");
  private SendableChooser<Command> autoChooser;
  private RobotSubsystems robot;
  private Timer timer = new Timer();
  private CustomBooleanLogger passedFirstNote = new CustomBooleanLogger(
      "Autonomous/" + getClass().getName() + "/passedFirstNote");

  public static boolean gotNotes[] = { false, false, false };

  public AutoStealOverkillLEFTDynamic(RobotSubsystems robot, SendableChooser<Command> autoChooser) {
    this.robot = robot;
    this.autoChooser = autoChooser;
    autoChooser.getSelected().getName();
    addCommands(
        new InstantCommand(() -> Robot.autonomousState = "STARTING_OVK_LEFT"),
        new InstantCommand(() -> timer.start()),
        this.autoChooser.getSelected()
            .alongWith(
                Commands.run(() -> gotNotes[0] = robot.shooter.hasPassedNoteThrough())
                    .until(() -> timer.get() > 5 || gotNotes[0]))
            .until(() -> timer.get() >= 5.2
                && !this.robot.shooter.hasNote()),
        new InstantCommand(() -> passedFirstNote.append(gotNotes[0])),
        either(new dontGotNote1(this.robot), new gotNote1(this.robot), () -> !this.robot.shooter.hasNote()),
        either(new goToNote2(this.robot), new GoToNote3(this.robot), () -> gotNotes[0]),
        new AutonomousGoToNote(robot),
        new AutonomousCollect(robot),
        AutoBuilder.followPath(path1),
        new AutonomousAimSwerveSpeaker(robot),
        new AutonomousShootInterpolation(robot));
  }

}
