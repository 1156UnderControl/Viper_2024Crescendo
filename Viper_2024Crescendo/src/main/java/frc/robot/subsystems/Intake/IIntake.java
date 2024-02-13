package frc.robot.subsystems.Intake;

public interface IIntake {
  void intake();

  void stopIntaking();

  void goToIntakePosition();

  void shootingPosition();

  void goToSecuredPosition();

  boolean isSecureForShooter();

  void expell();

  boolean isAtSetpoint();

  void setManualPivotPosition(double goal);
}
