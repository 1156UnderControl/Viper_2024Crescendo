package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToIntakeIntake;
import frc.robot.commands.Shooter.CollectToShooter;
public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(RobotSubsystems robot) {
    addCommands(
        new GoToIntakeIntake(robot),
        new CollectToShooter(robot),
        Commands.runOnce(() -> robot.intake.stopIntaking()));
  }
}
