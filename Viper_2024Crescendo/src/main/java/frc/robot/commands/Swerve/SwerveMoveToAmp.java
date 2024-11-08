package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Joysticks.controllers.OneController;
import frc.robot.subsystems.Swerve.ISwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveMoveToAmp extends Command {
  private ISwerve swerve;
  private OneController controller = OneController.getInstance();
  boolean angleAligned;

  public SwerveMoveToAmp(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    angleAligned = false;
  }

  @Override
  public void initialize() {
    angleAligned = false;
  }

  @Override
  public void execute() {
    if (this.swerve.isAtAmpAngle()) {
      angleAligned = true;
    }
    if (angleAligned) {
      swerve.driveToAmp();
    } else {
      swerve.driveAlignAngleButton();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return this.swerve.isAtAmpPosition() || controller.isForcingDriverControl() || this.swerve.isAlignedWithAmp();
  }
}
