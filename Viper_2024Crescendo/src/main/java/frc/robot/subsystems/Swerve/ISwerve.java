package frc.robot.subsystems.Swerve;

public interface ISwerve {
  void driveAlignAngleButton();

  void driveAlignAngleJoy();

  void driveAlignSpeaker();

  void driveAlignNote();

  void driveToAmp();

  boolean isSwerveAlignedWithSpeaker();

  boolean isSwerveInAutoShootZone();

  boolean isSwerveInVelocityToAutoShoot();

  boolean isSwerveInVelocityToShoot();
}
