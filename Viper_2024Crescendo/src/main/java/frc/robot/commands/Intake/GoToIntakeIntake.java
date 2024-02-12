package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class GoToIntakeIntake extends Command {
  private IntakeSubsystem intake;

  public GoToIntakeIntake(IntakeSubsystem intake) {
    this.intake = intake;
    this.addRequirements(intake);
  }

  @Override
  public void initialize() {
    this.intake.intake();
  }

  @Override
  public void execute() {
    this.intake.intakePosition();
  }

  @Override
  public boolean isFinished() {
    return this.intake.secureForShooter();
  }
}
