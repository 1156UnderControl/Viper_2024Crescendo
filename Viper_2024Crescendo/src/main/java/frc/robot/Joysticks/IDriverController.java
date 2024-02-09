package frc.robot.Joysticks;

public interface IDriverController {
  double getXtranslation();

  double getYtranslation();

  double getCOS_Joystick();

  double getSIN_Joystick();

  double getThrottle();

  boolean turboActivate();

  boolean notUsingJoystick();

}
