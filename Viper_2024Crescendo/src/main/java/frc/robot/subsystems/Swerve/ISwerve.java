package frc.robot.subsystems.Swerve;

public interface ISwerve {
  void driveAlignAngleButton();

  void driveAlignAngleJoy();

  void driveAlignSpeaker();

  void driveAlignNote();

  boolean isSwerveAlignedWithSpeaker();

  boolean isSwerveInAutoShootZone();

  boolean isSwerveInVelocityToAutoShoot();

  boolean isSwerveInVelocityToShoot();
}
