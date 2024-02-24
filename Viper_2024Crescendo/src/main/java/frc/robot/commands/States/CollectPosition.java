package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToIntakeIntake;
import frc.robot.commands.Shooter.CollectToShooter;
import frc.robot.subsystems.LEDs.LedColor;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(RobotSubsystems robot) {
    addCommands(
        new GoToIntakeIntake(robot),
        new CollectToShooter(robot),
        new InstantCommand(() -> robot.led.setBlink(LedColor.GREEN, 3, LedColor.WHITE)),
        Commands.runOnce(() -> robot.intake.stopIntaking()));
  }
}
