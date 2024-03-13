package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.ISwerve;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveMoveToAmp extends Command {
    private ISwerve swerve;

    public SwerveMoveToAmp(SwerveSubsystem swerve) {
      this.swerve = swerve;
      addRequirements(swerve);
    }
  
    @Override
    public void execute() {
      swerve.driveToAmp();
    }
  
    @Override
    public void end(boolean interrupted) {
    }
  
    @Override
    public boolean isFinished() {
      return this.swerve.isAtAmpPosition();
    }
  }
