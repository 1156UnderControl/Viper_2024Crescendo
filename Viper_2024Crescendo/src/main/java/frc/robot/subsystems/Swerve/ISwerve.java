package frc.robot.subsystems.Swerve;

public interface ISwerve {
  void driveAlignAngleButton();

  void driveAlignAngleJoy();

  void driveAlignSpeaker();

  void driveAlignShootOverStage();

  void driveAlignNote();

  void driveToAmp();

  void alignWithAmpAprilTag();

  void setAngleToAmp();

  void overrideRotationAuto(boolean override);

  double getDistanceToNearestAmp();

  boolean isSwerveAlignedWithSpeaker();

  boolean isAtAmpAngle();

  boolean isSwerveInAutoShootZone();

  boolean isSwerveInVelocityToAutoShoot();

  boolean isSwerveInVelocityToShoot();

  boolean isAtAmpPosition();

  boolean isAlignedWithAmp();
}
