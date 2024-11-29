package frc.robot.Joysticks.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IOneController {
  double getXtranslation();

  double getYtranslation();

  double getCOS_Joystick();

  double getSIN_Joystick();

  double getHeadingY();

  double getHeadingX();

  boolean turboActivate();

  boolean notUsingJoystick();

  boolean rotateLeft();

  boolean rotateRight();

  Trigger resetGyro();

  boolean isForcingDriverControl();

  Trigger getIntake();

  Trigger getSecured();

  Trigger getPrepareAmp();

  Trigger getPrepareSpeaker();

  Trigger climb();

  Trigger release();

  Trigger getDoNotShoot();

  Trigger falsePositive();

  Trigger falseNegative();

  Trigger expell();
}
