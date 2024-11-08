package frc.robot.Joysticks;

import frc.robot.Joysticks.type_controllers.IController;
import frc.robot.Joysticks.type_controllers.TypePS5Controller;
import frc.robot.Joysticks.type_controllers.TypeXboxController;

public class RobotController {

  public IController typeController;

  public RobotController(IController typeController, int controllerID) {
    this.typeController = typeController;
    this.typeController = new TypePS5Controller(controllerID);
    this.typeController = new TypeXboxController(controllerID);
  }
}
