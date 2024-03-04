package frc.robot.subsystems.Shooter;

public interface IShooter {
  void intake();

  void intakeFromHP();

  void intakeAuto();

  void stopIntaking();

  void stopFeeder();

  void goToIntakePosition();

  boolean hasNote();

  boolean getNoteSensor();

  void prepareSpeakerShot();

  void prepareAmpShot();

  void goToSecuredPosition();

  void shoot();

  void cruise();

  void expell();

  boolean hasShooted();

  boolean isSecureForIntake();

  boolean isPivotAtSetpoint();

  boolean isFlywheelAtSetpoint();

  void setManualPivotPosition(double goal);

  void setHasNoteFalse();

  void setHasNoteTrue();
}
