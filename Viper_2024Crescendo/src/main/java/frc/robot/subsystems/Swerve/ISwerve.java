package frc.robot.subsystems.Swerve;

public interface ISwerve {
  void driveAlignAngleButton();

  void driveAlignAngleJoy();

  void driveAlignSpeaker();

  void driveAlignNote();

  void driveToAmp();

  void alignWithAmpAprilTag();

  void setAngleToAmp();

  void overrideRotationAuto(boolean Override);

  double getDistanceToNearestAmp();

  boolean isSwerveAlignedWithSpeaker();

  boolean isAlignedWithAngle();

  boolean isSwerveInAutoShootZone();

  boolean isSwerveInVelocityToAutoShoot();

  boolean isSwerveInVelocityToShoot();

  boolean isAtAmpPosition();

  boolean isAlignedWithAmp();
}
