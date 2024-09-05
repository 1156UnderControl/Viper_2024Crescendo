package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Intake.GoToShootIntake;
import frc.robot.commands.Shooter.ShootSpeaker;
import frc.robot.subsystems.LEDs.LedColor;

public class ShootSpeakerPosition extends SequentialCommandGroup {
  ControlBoard mControlBoard = ControlBoard.getInstance();

  public ShootSpeakerPosition(RobotSubsystems robot) {
    addCommands(
        new InstantCommand(() -> RobotSubsystems.shootFinished = false),
        new InstantCommand(() -> robot.preparingShoot = true),
        new InstantCommand(() -> robot.intake.stopIntaking()),
        new InstantCommand(() -> robot.led.setSolidColor(LedColor.YELLOW)),
        new GoToShootIntake(robot),
        new ShootSpeaker(robot),
        new InstantCommand(() -> robot.preparingShoot = false),
        Commands.runOnce(() -> robot.shooter.shoot(), robot),
        new InstantCommand(() -> robot.led.setBlink(LedColor.GREEN, 3)),
        Commands.waitSeconds(1),
        new InstantCommand(() -> RobotSubsystems.shootFinished = true));
  }
}
