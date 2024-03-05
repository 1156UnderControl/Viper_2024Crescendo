package frc.robot.Joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;

public class DriverController implements IDriverController {

  private static DriverController mInstance = null;

  public static DriverController getInstance() {
    if (mInstance == null) {
      mInstance = new DriverController();
    }

    return mInstance;
  }

  final CommandXboxController driverController;

  private DriverController() {
    driverController = new CommandXboxController(0);
  }

  @Override
  public double getXtranslation() {
    if (turboActivate()) {
      return -MathUtil.applyDeadband(driverController.getLeftX(), JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(driverController.getLeftX(), JoystickConstants.DEADBAND) * 0.6;
  }

  @Override
  public double getYtranslation() {
    if (turboActivate()) {
      return -MathUtil.applyDeadband(driverController.getLeftY(), JoystickConstants.DEADBAND);
    }
    return -MathUtil.applyDeadband(driverController.getLeftY(), JoystickConstants.DEADBAND) * 0.6;
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
  public Trigger rotateLeft() {
    return driverController.leftBumper();
  }

  @Override
  public Trigger rotateRight() {
    return driverController.rightBumper();
  }

  @Override
  public boolean isForcingDriverControl() {
    return 0.2 < driverController.getLeftTriggerAxis();
  }

}
