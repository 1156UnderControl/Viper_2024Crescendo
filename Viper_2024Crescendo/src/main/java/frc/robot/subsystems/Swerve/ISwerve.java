package frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Joysticks.controllers.OneController;

public interface ISwerve {
  void setModulesToCoast();

  void setModulesToBrake();

  void driveAlignAngleButton();

  void driveAlignAngleJoy();

  void driveAlignSpeaker();

  void driveAlignSpeakerSlow();

  void alignSpeaker();

  void aimSwerve(double angleToAim);

  InstantCommand setButtonHeading(OneController controller, SwerveSubsystem swerve);

  boolean isAtTargetHeading();

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
