package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToIntakeIntake;
import frc.robot.commands.Shooter.CollectToShooter;
import frc.robot.subsystems.LEDs.LedColor;
import frc.robot.subsystems.LEDs.LedSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(RobotSubsystems robot, LedSubsystem led) {
    addCommands(
        new GoToIntakeIntake(robot, led),
        new CollectToShooter(robot),
        new InstantCommand(() -> led.setSolidColor(LedColor.GREEN), led),
        Commands.runOnce(() -> robot.intake.stopIntaking()));
  }
}
