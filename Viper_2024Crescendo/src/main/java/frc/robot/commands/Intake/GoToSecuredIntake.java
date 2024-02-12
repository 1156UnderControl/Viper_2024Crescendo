package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class GoToSecuredIntake extends Command {
  private IntakeSubsystem intake;

  public GoToSecuredIntake(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);

  }

  @Override
  public void initialize() {
    this.intake.securedPosition();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
