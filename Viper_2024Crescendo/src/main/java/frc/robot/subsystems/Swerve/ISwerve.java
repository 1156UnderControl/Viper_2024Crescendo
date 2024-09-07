package frc.robot.subsystems.Swerve;

public interface ISwerve {
  void setModulesToCoast();

  void setModulesToBrake();

  void driveAlignAngleButton();

  void driveAlignAngleJoy();

  void driveAlignSpeaker();

  void driveAlignSpeakerSlow();

  void alignSpeaker();

  void driveAlignShootOverStage();

  void driveAlignNote();

  void driveToNote();

  void driveToAmp();

  void alignWithAmpAprilTag();

  void setAngleToAmp();

  void overrideRotationAuto(boolean override);

  double getDistanceToNearestAmp();

  double distanceFromSpeakerMoving();

  boolean isSwerveAlignedWithSpeaker();

  boolean isAtAmpAngle();

  boolean isSwerveInAutoShootZone();

  boolean isSwerveInVelocityToAutoShoot();

  boolean isSwerveInVelocityToShoot();

  boolean isAtAmpPosition();

  boolean isAlignedWithAmp();
}
