package frc.robot.Joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IDriverController {
  double getXtranslation();

  double getYtranslation();

  double getCOS_Joystick();

  double getSIN_Joystick();

  boolean turboActivate();

  boolean notUsingJoystick();

  Trigger setHeadingFront();

  Trigger setHeadingRight();

  Trigger setHeadingBack();

  Trigger setHeadingLeft();

  Trigger setHeading45Left();

  Trigger setHeading45Right();

  boolean isForcingDriverControl();
}
