package frc.robot.subsystems.Climber;

public interface IClimber {
  void release();

  void climb();

  void stop();

  boolean isNearLimit();

  boolean isAtSetpoint();
}
