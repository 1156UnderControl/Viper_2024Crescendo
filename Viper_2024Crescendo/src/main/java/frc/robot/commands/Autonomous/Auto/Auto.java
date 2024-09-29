
package frc.robot.commands.Autonomous.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;

public class Auto extends SequentialCommandGroup {
  private SendableChooser<Command> autoChooser;
  private RobotSubsystems robot;

  public static boolean gotNotes[] = { false, false, false };

  public Auto(RobotSubsystems robot, SendableChooser<Command> autoChooser) {
    this.robot = robot;
    this.autoChooser = autoChooser;
    switch (this.autoChooser.getSelected().getName()) {
      case "Steal Overkill Left (Red) Dynamic":
        addCommands(new AutoStealOverkillLEFTDynamic(this.robot, this.autoChooser));
        break;
      case "Steal Overkill RIGHT (Red) Dynamic":
        addCommands(new AutoStealOverkillRIGHTDynamic(this.robot, this.autoChooser));
        break;
      default:
        addCommands(this.autoChooser.getSelected());
        break;
    }
  }

}
