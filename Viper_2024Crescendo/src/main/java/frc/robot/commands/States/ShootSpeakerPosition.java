package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Intake.GoToShootIntake;
import frc.robot.commands.Shooter.ShootSpeaker;

public class ShootSpeakerPosition extends SequentialCommandGroup {
  ControlBoard mControlBoard = ControlBoard.getInstance();

  public ShootSpeakerPosition(RobotSubsystems robot) {
    addCommands(
        new InstantCommand(() -> RobotSubsystems.shootFinished = false),
        new GoToShootIntake(robot),
        new ShootSpeaker(robot),
        Commands.runOnce(() -> robot.shooter.shoot(), robot),
        Commands.waitSeconds(1),
        new InstantCommand(() -> RobotSubsystems.shootFinished = true));
  }
}
