package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.controllers.OneController;

public class ExpellPosition extends SequentialCommandGroup {
  OneController controller = OneController.getInstance();

  public ExpellPosition(RobotSubsystems robot) {
    addCommands(
        Commands.runOnce(() -> robot.intake.expell(), robot),
        Commands.waitUntil(() -> !controller.expell().getAsBoolean()));
  }
}
