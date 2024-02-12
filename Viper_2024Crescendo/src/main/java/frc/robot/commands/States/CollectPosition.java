package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.GoToIntakeIntake;
import frc.robot.commands.Shooter.CollectToShooter;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(ShooterSubsystem shooter, IntakeSubsystem intake) {
    addCommands(
        new GoToIntakeIntake(intake),
        new CollectToShooter(shooter),
        Commands.runOnce(() -> intake.stopIntaking()));
  }
}
