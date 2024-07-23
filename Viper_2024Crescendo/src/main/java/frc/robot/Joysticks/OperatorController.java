package frc.robot.Joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController implements IOperatorController {
  private static OperatorController mInstance = null;

  public static OperatorController getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorController();
    }

    return mInstance;
  }

  CommandPS5Controller controller;

  private OperatorController() {
    controller = new CommandPS5Controller(1);
  }

  @Override
  public Trigger getIntake() {
    return controller.button(2);
  }

  @Override
  public Trigger getIntakeFromHP() {
    return controller.R2();
  }

  @Override
  public Trigger getSecured() {
    return controller.button(4);
  }

  @Override
  public Trigger getPrepareAmp() {
    return controller.L1();
  }

  @Override
  public Trigger getPrepareSpeaker() {
    return controller.R1();
  }

  @Override
  public Trigger getShoot() {
    return controller.L2();
  }

  @Override
  public Trigger climb() {
    return controller.button(9);
  }

  @Override
  public Trigger release() {
    return controller.button(10);
  }

  @Override
  public Trigger getDoNotShoot() {
    return controller.button(3);
  }

  @Override
  public Trigger falsePositive() {
    return controller.R3();

  }

  @Override
  public Trigger falseNegative() {
    return controller.L3();
  }

  @Override
  public Trigger expell() {
    return controller.button(1);
  }

}
