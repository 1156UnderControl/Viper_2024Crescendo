package frc.robot.subsystems.Shooter;

public interface IShooter {
  void intake();

  void intakeAuto();

  void stopIntaking();

  void goToIntakePosition();

  boolean hasNote();

  void prepareSpeakerShot();

  void prepareAmpShot();

  void goToSecuredPosition();

  void shoot();

  void cruise();

  boolean hasShooted();

  boolean isSecureForIntake();

  boolean isAtSetpoint();

  void setManualPivotPosition(double goal);
}
