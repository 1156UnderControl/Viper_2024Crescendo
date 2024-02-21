package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AutonomousVisionUpdate extends Command {
  SwerveSubsystem swerve;

  public AutonomousVisionUpdate(SwerveSubsystem swerve) {
    this.swerve = swerve;
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "VISION UPDATE";
    this.swerve.addVisionReading();
  }

  @Override
  public void execute() {
    this.swerve.addVisionReading();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
