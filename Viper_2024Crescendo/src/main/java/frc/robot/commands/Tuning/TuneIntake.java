package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class TuneIntake extends Command {
  private IntakeSubsystem intake;

  public TuneIntake(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.tuningState(SmartDashboard.getNumber("Position Manual Setpoint", 20));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
