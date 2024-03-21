package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveAutoAmpRoutine extends SequentialCommandGroup {

  public SwerveAutoAmpRoutine(SwerveSubsystem swerve) {
    addCommands(
        // new SwerveDetectClosenessToAmp(swerve),
        // new InstantCommand(() -> swerve.setAngleToAmp()),
        // new SwerveMoveToAmp(swerve),
        new SwerveAlignWithAmp(swerve));
  }
}
