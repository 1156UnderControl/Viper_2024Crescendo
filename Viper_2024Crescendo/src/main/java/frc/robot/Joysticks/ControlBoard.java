package frc.robot.Joysticks;

public class ControlBoard implements IDriverController, IOperatorController {
  private static ControlBoard mInstance = null;

  public static ControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new ControlBoard();
    }

    return mInstance;
  }

  IDriverController mDriverController;
  IOperatorController mOperatorController;

  private ControlBoard() {
    mDriverController = DriverController.getInstance();
    mOperatorController = OperatorController.getInstance();
  }

}
