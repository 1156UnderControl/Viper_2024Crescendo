
package frc.robot.commands.Autonomous.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;

public class Auto extends SequentialCommandGroup {
  private SendableChooser<Command> autoChooser;
  private RobotSubsystems robot;
  private Timer timer = new Timer();

  public static boolean gotNotes[] = { false, false, false };

  public Auto(RobotSubsystems robot, SendableChooser<Command> autoChooser) {
    this.robot = robot;
    this.autoChooser = autoChooser;
    autoChooser.getSelected().getName();
    switch (autoChooser.getSelected().getName()) {
      case "Steal Overkill Left (Red) Dynamic":
        addCommands(new AutoStealOverkillDynamic(robot, autoChooser));
        break;
      default:
        addCommands(autoChooser.getSelected());
        break;
    }
  }

}
