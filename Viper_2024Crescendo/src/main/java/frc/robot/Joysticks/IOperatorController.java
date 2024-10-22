package frc.robot.Joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IOperatorController {
  Trigger getIntake();

  Trigger getIntakeFromHP();

  Trigger getSecured();

  Trigger getPrepareAmp();

  Trigger getPrepareSpeaker();

  Trigger getShoot();

  Trigger climb();

  Trigger release();

  Trigger getDoNotShoot();

  Trigger falsePositive();

  Trigger falseNegative();

  Trigger expell();
}
