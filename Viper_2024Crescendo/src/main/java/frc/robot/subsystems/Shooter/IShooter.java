package frc.robot.subsystems.Shooter;

public interface IShooter {
  void intake();

  void intakeFromHP();

  void intakeAuto();

  void passThrough();

  void passThroughWithoutPosition();

  void stopIntaking();

  void stopFeeder();

  void goToIntakePosition();

  boolean hasNote();

  boolean getNoteSensor();

  boolean getAnyNoteSensor();

  void prepareSpeakerShot();

  void prepareAmpShot();

  void prepareShootOverStage();

  void goToSecuredPosition();

  void shoot();

  void cruise();

  void expell();

  boolean hasShooted();

  boolean isSecureForIntake();

  boolean isPivotAtSetpoint();

  boolean isPivotAtSetpointForAutoShoot();

  boolean isFlywheelAtSetpoint();

  void setManualPivotPosition(double goal);

  void setHasNoteFalse();

  void setHasNoteTrue();
}
