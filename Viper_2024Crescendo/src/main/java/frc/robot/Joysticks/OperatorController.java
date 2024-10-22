package frc.robot.Joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Joysticks.controllers.TypeXboxController;

public class OperatorController implements IOperatorController {
  private static OperatorController mInstance = null;

  RobotController controller;
  TypeXboxController controllerType;

  public static OperatorController getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorController();
    }
    return mInstance;
  }

  private OperatorController() {
    controller = new RobotController(controllerType, 1);
  }

  @Override
  public Trigger getIntake() {
    return controller.typeController.buttomDown();
  }

  @Override
  public Trigger getSecured() {
    return controller.typeController.buttomUp();
  }

  @Override
  public Trigger getPrepareAmp() {
    return controller.typeController.L1();
  }

  @Override
  public Trigger getPrepareSpeaker() {
    return controller.typeController.R1();
  }

  @Override
  public Trigger getShoot() {
    return controller.typeController.R2();
  }

  @Override
  public Trigger climb() {
    return controller.typeController.share();
  }

  @Override
  public Trigger release() {
    return controller.typeController.options();
  }

  @Override
  public Trigger falsePositive() {
    return controller.typeController.R3();
  }

  @Override
  public Trigger falseNegative() {
    return controller.typeController.R3();
  }

  @Override
  public Trigger expell() {
    return controller.typeController.buttomLeft();
  }
}
