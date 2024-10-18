package frc.robot.Joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController implements IOperatorController {
  private static OperatorController mInstance = null;

  public static OperatorController getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorController();
    }

    return mInstance;
  }

  CommandXboxController controller;

  private OperatorController() {
    controller = new CommandXboxController(1);
  }

  @Override
  public Trigger getIntake() {

    return controller.a();
  }

  @Override
  public Trigger getIntakeFromHP() {
    return controller.rightTrigger(0.2);
  }

  @Override
  public Trigger getSecured() {
    return controller.y();
  }

  @Override
  public Trigger getPrepareAmp() {
    return controller.leftBumper();
  }

  @Override
  public Trigger getPrepareSpeaker() {
    return controller.rightBumper();
  }

  @Override
  public Trigger getShoot() {
    return controller.leftTrigger(0.2);
  }

  @Override
  public Trigger climb() {
    return controller.back();
  }

  @Override
  public Trigger release() {
    return controller.start();
  }

  @Override
  public Trigger getDoNotShoot() {
    return controller.b();
  }

  @Override
  public Trigger falsePositive() {
    return controller.rightStick();
  }

  @Override
  public Trigger falseNegative() {
    return controller.leftStick();
  }

  @Override
  public Trigger expell() {
    return controller.x();
  }

}
