package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveAlignWithAmp extends SequentialCommandGroup {

  public SwerveAlignWithAmp(SwerveSubsystem swerve) {
    addCommands(
        new SwerveDetectClosenessToAmp(swerve),
        new SwerveMoveToAmp(swerve));
  }
}
