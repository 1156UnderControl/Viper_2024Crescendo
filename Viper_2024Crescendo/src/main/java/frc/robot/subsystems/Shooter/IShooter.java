package frc.robot.subsystems.Shooter;

public interface IShooter {
  void intake();

  boolean hasNote();

  void prepareSpeakerShot();

  void prepareAmpShot();

  void securedPosition();

  void shoot();

  boolean hasShooted();

  boolean isSecureForIntake();

  boolean atSetpoint();
}
