package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Intake.GoToShootIntake;
import frc.robot.commands.Shooter.ShootAmp;
import frc.robot.subsystems.LEDs.LedColor;

public class ShootAmpPosition extends SequentialCommandGroup {
  ControlBoard mControlBoard = ControlBoard.getInstance();

  public ShootAmpPosition(RobotSubsystems robot) {
    addCommands(
        new InstantCommand(() -> RobotSubsystems.shootFinished = false),
        new InstantCommand(() -> robot.preparingShoot = true),
        new InstantCommand(() -> robot.led.setSolidColor(LedColor.YELLOW)),
        new GoToShootIntake(robot),
        new ShootAmp(robot),
        new InstantCommand(() -> robot.led.setSolidColor(LedColor.GREEN)),
        Commands.waitUntil(mControlBoard.getShoot()),
        new InstantCommand(() -> robot.preparingShoot = false),
        Commands.runOnce(() -> robot.shooter.shoot(), robot),
        new InstantCommand(() -> robot.led.setBlink(LedColor.GREEN, 3)),
        Commands.waitSeconds(1),
        new InstantCommand(() -> RobotSubsystems.shootFinished = true));
  }
}
