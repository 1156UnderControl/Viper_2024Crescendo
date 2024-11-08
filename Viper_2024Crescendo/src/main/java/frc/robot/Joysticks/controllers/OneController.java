package frc.robot.Joysticks.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Joysticks.RobotController;
import frc.robot.Joysticks.type_controllers.TypeXboxController;

public class OneController implements IOneController {
  private static OneController mInstance = null;

  public static OneController getInstance() {
    if (mInstance == null) {
      mInstance = new OneController();
    }

    return mInstance;
  }

  final CommandXboxController driverController;
  final TypeXboxController xboxController;
  final RobotController controller;

  private OneController() {
    xboxController = new TypeXboxController(0);
    driverController = new CommandXboxController(0);
    controller = new RobotController(xboxController, 0);
  }

  @Override
  public double getXtranslation() {
    if (turboActivate()) {
      return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(driverController.getLeftX()),
          JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(driverController.getLeftX()),
        JoystickConstants.DEADBAND) * 0.6;
  }

  @Override
  public double getYtranslation() {
    if (turboActivate()) {
      return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(driverController.getLeftY()),
          JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(driverController.getLeftY()),
        JoystickConstants.DEADBAND) * 0.6;
  }

  @Override
  public Trigger setHeadingBack() {
    return driverController.a();
  }

  @Override
  public Trigger setHeadingFront() {
    return driverController.y();
  }

  @Override
  public Trigger setHeadingLeft() {
    return driverController.x();
  }

  @Override
  public Trigger setHeadingRight() {
    return driverController.b();
  }

  @Override
  public double getCOS_Joystick() {
    return -driverController.getRightX();
  }

  @Override
  public double getSIN_Joystick() {
    return -driverController.getRightY();
  }

  @Override
  public boolean turboActivate() {
    return 0.2 < driverController.getRightTriggerAxis();
  }

  @Override
  public boolean notUsingJoystick() {
    return frc.Java_Is_UnderControl.Util.Util.inRange(getCOS_Joystick(),
        -JoystickConstants.DEADBAND,
        0.2)
        && frc.Java_Is_UnderControl.Util.Util.inRange(getSIN_Joystick(), -JoystickConstants.DEADBAND,
            JoystickConstants.DEADBAND);
  }

  @Override
  public boolean rotateLeft() {
    return controller.typeController.L1();
  }

  @Override
  public boolean rotateRight() {
    return controller.typeController.R1();
  }

  @Override
  public boolean isForcingDriverControl() {
    return 0.2 < driverController.getLeftTriggerAxis();
  }

  @Override
  public Trigger resetGyro() {
    return driverController.back();
  }

  private double performAllianceSpeedDirectionCorrection(Double value) {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
        : DriverStation.Alliance.Red;
    if (alliance == Alliance.Red) {
      return -value;
    }
    return value;
  }

  @Override
  public Trigger getIntake() {
    return controller.typeController.dPadRight();
  }

  @Override
  public Trigger getSecured() {
    return controller.typeController.buttomDown();
  }

  @Override
  public Trigger getPrepareAmp() {
    return controller.typeController.dPadLeft();
  }

  @Override
  public Trigger getPrepareSpeaker() {
    return controller.typeController.dPadDown();
  }

  @Override
  public Trigger getShoot() {
    return controller.typeController.dPadUp();
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
  public Trigger getDoNotShoot() {
    return controller.typeController.buttomRight();
  }

  @Override
  public Trigger falsePositive() {
    return controller.typeController.R3();
  }

  @Override
  public Trigger falseNegative() {
    return controller.typeController.L3();
  }

  @Override
  public Trigger expell() {
    return controller.typeController.buttomLeft();
  }

}
