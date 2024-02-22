package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToIntakeIntake;
import frc.robot.commands.Shooter.CollectToShooter;
import frc.robot.commands.Swerve.SwerveAlignNote;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(RobotSubsystems robot, SwerveSubsystem swerve) {
    addCommands(
        new GoToIntakeIntake(robot),
        new CollectToShooter(robot),
        new SwerveAlignNote(swerve),
        Commands.runOnce(() -> robot.intake.stopIntaking()));
  }
}
