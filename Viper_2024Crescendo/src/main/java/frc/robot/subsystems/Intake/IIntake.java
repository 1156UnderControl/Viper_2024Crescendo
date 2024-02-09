package frc.robot.subsystems.Intake;

public interface IIntake {
  void intake();

  void freewayShooter();

  void securedPosition();

  boolean secureForShooter();

  void expell();

  void hasCollected();
}
