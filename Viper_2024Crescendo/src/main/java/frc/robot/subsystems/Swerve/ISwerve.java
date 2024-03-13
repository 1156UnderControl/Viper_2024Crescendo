package frc.robot.subsystems.Swerve;

public interface ISwerve {
  void driveAlignAngleButton();

  void driveAlignAngleJoy();

  void driveAlignSpeaker();

  void driveAlignNote();

  void driveToAmp();

  void alignWithAmpAprilTag();

  double getDistanceToNearestAmp();

  boolean isSwerveAlignedWithSpeaker();

  boolean isSwerveInAutoShootZone();

  boolean isSwerveInVelocityToAutoShoot();

  boolean isSwerveInVelocityToShoot();

  boolean isAtAmpPosition();

  boolean isAlignedWithAmp();
}
