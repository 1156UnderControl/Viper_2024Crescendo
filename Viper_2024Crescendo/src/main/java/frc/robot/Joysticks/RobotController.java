package frc.robot.Joysticks;

import frc.robot.Joysticks.controllers.IController;
import frc.robot.Joysticks.controllers.TypePS5Controller;
import frc.robot.Joysticks.controllers.TypeXboxController;

public class RobotController {

  IController typeController;

  public RobotController(IController typeController, int controllerID) {
    this.typeController = typeController;
    this.typeController = new TypePS5Controller(controllerID);
    this.typeController = new TypeXboxController(controllerID);
  }
}
