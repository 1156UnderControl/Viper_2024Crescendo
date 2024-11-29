package frc.robot.Joysticks;

import frc.robot.Joysticks.type_controllers.IController;

public class RobotController {

  public IController typeController;

  public RobotController(IController typeController, int controllerID) {
    this.typeController = typeController;
  }
}
