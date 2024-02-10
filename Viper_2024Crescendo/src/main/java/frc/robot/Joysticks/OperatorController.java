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
    controller = new CommandXboxController(0);
  }

  @Override
  public Trigger getIntake() {

    return controller.a();
  }

  @Override
  public Trigger getSecured() {
    return controller.y();
  }
}
