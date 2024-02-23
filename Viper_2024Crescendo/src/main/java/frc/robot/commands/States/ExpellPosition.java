package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;

public class ExpellPosition extends SequentialCommandGroup {
  ControlBoard mControlBoard = ControlBoard.getInstance();

  public ExpellPosition(RobotSubsystems robot) {
    addCommands(
        Commands.runOnce(() -> robot.intake.expell(), robot),
        Commands.runOnce(() -> robot.shooter.expell(), robot),
        new InstantCommand(() -> robot.shooter.setHasNoteFalse()),
        Commands.waitUntil(() -> !mControlBoard.expell().getAsBoolean()));
  }
}
