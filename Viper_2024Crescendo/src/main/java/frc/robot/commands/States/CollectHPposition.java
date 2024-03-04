package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToIntakeHP;
import frc.robot.commands.Shooter.CollectToShooterHP;
import frc.robot.subsystems.LEDs.LedColor;

public class CollectHPposition extends SequentialCommandGroup {

  public CollectHPposition(RobotSubsystems robot) {
    addCommands(
        new GoToIntakeHP(robot),
        new CollectToShooterHP(robot),
        new InstantCommand(() -> robot.led.setBlink(LedColor.GREEN, 3, LedColor.WHITE)),
        Commands.runOnce(() -> robot.intake.stopIntaking()));
  }
}
