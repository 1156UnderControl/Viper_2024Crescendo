package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.ISwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveAlignWithAmp extends Command {
  private ISwerve swerve;

  public SwerveAlignWithAmp(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    this.swerve.driveAlignAngleButton();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAlignedWithAmp();
  }
}
