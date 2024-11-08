package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.controllers.OneController;
import frc.robot.commands.Intake.GoToShootIntake;
import frc.robot.commands.Shooter.ShootAmp;
import frc.robot.commands.Swerve.SwerveAutoAmpRoutine;
import frc.robot.subsystems.LEDs.LedColor;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ShootAmpPosition extends SequentialCommandGroup {
  OneController controller = OneController.getInstance();

  public ShootAmpPosition(RobotSubsystems robot, SwerveSubsystem swerve) {
    addCommands(
        new InstantCommand(() -> RobotSubsystems.shootFinished = false),
        new InstantCommand(() -> robot.intake.stopIntaking()),
        new InstantCommand(() -> robot.preparingShoot = true),
        new InstantCommand(() -> robot.led.setSolidColor(LedColor.YELLOW)),
        new GoToShootIntake(robot),
        new ParallelCommandGroup(new ShootAmp(robot), new SwerveAutoAmpRoutine(swerve)),
        new InstantCommand(() -> robot.led.setSolidColor(LedColor.GREEN)),
        Commands.runOnce(() -> robot.shooter.shoot(), robot),
        new InstantCommand(() -> robot.led.setBlink(LedColor.GREEN, 3)),
        Commands.waitSeconds(0.45),
        new InstantCommand(() -> RobotSubsystems.shootFinished = true));
  }
}
