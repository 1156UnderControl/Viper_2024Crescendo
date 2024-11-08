package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Joysticks.controllers.OneController;
import frc.robot.subsystems.Swerve.ISwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveAlignWithAmp extends Command {
  private Timer timer = new Timer();
  private ISwerve swerve;
  private OneController controller = OneController.getInstance();

  public SwerveAlignWithAmp(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    this.swerve.driveAlignAngleButton();
  }

  @Override
  public void end(boolean interrupted) {
    this.swerve.driveAlignAngleButton();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return (this.swerve.isAlignedWithAmp() && timer.get() > 1.5) || this.controller.getShoot().getAsBoolean();
  }
}
