package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToIntakeIntake;
import frc.robot.commands.Shooter.CollectToShooter;
import frc.robot.commands.Swerve.SwerveAlignNote;
import frc.robot.subsystems.LEDs.LedSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(RobotSubsystems robot, SwerveSubsystem swerve, LedSubsystem led) {
    addCommands(
        new GoToIntakeIntake(robot, led),
        new CollectToShooter(robot),
        new SwerveAlignNote(swerve),
        new InstantCommand(() -> led.setGreen(), led),
        Commands.runOnce(() -> robot.intake.stopIntaking()));
  }
}
