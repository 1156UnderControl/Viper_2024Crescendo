package frc.robot.Joysticks.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Joysticks.type_controllers.TypeXboxController;

public class OneController implements IOneController {
  private static OneController mInstance = null;
  public static boolean climbMode = false;

  public static OneController getInstance() {
    if (mInstance == null) {
      mInstance = new OneController();
    }

    return mInstance;
  }

  final TypeXboxController controller;

  private OneController() {
    controller = new TypeXboxController(0);
  }

  @Override
  public double getXtranslation() {
    if (turboActivate()) {
      return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(controller.leftX()),
          JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(controller.leftX()),
        JoystickConstants.DEADBAND) * 0.6;
  }

  @Override
  public double getYtranslation() {
    if (turboActivate()) {
      return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(controller.leftX()),
          JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(performAllianceSpeedDirectionCorrection(controller.leftX()),
        JoystickConstants.DEADBAND) * 0.6;
  }

  @Override
  public double getHeadingY() {
    return controller.rightY();
  }

  @Override
  public double getHeadingX() {
    return controller.rightX();
  }

  @Override
  public double getCOS_Joystick() {
    return -controller.rightX();
  }

  @Override
  public double getSIN_Joystick() {
    return -controller.rightY();
  }

  @Override
  public boolean turboActivate() {
    return 0.2 < controller.leftY();
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
    return controller.L1();
  }

  @Override
  public boolean rotateRight() {
    return controller.R1();
  }

  @Override
  public boolean isForcingDriverControl() {
    return 0.2 < controller.leftY();
  }

  @Override
  public Trigger resetGyro() {
    return controller.share();
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
    return controller.buttomDown();
  }

  @Override
  public Trigger getSecured() {
    return controller.buttomRight();
  }

  @Override
  public Trigger getPrepareAmp() {
    return controller.L2();
  }

  @Override
  public Trigger getPrepareSpeaker() {
    return controller.R2();
  }

  @Override
  public Trigger climb() {
    if (OneController.climbMode) {
      return controller.dPadUp();
    } else {
      return new Trigger(() -> false);
    }
  }

  @Override
  public Trigger release() {
    if (OneController.climbMode) {
      return controller.dPadDown();
    } else {
      return new Trigger(() -> false);
    }
  }

  @Override
  public Trigger getDoNotShoot() {
    return controller.buttomRight();
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
    return controller.buttomLeft();
  }

}
