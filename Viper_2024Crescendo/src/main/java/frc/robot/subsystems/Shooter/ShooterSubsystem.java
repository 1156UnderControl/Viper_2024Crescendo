package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem mInstance = null;

  public static ShooterSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ShooterSubsystem();
    }

    return mInstance;
  }

  private ShooterSubsystem() {

  }
}
