package frc.robot.Joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;

public class DriverController implements IDriverController {

  private static DriverController mInstance = null;

  public static DriverController getInstance() {
    if (mInstance == null) {
      mInstance = new DriverController();
    }

    return mInstance;
  }

  final XboxController driverXbox;

  private DriverController() {
    driverXbox = new XboxController(0);
  }

  @Override
  public double getXtranslation() {
    return -MathUtil.applyDeadband(driverXbox.getLeftY(), JoystickConstants.DEADBAND);
  }

  @Override
  public double getYtranslation() {
    return -MathUtil.applyDeadband(driverXbox.getLeftX(), JoystickConstants.DEADBAND);
  }

  @Override
  public double getCOS_Joystick() {
    return -driverXbox.getRightX();
  }

  @Override
  public double getSIN_Joystick() {
    return -driverXbox.getRightY();
  }

  @Override
  public double getThrottle() {
    return driverXbox.getRightTriggerAxis();
  }

  @Override
  public boolean turboActivate() {
    return 0.2 < driverXbox.getLeftTriggerAxis();
  }

  @Override
  public boolean notUsingJoystick() {
    return frc.Java_Is_UnderControl.Util.Util.inRange(getCOS_Joystick(),
        -JoystickConstants.DEADBAND,
        0.2)
        && frc.Java_Is_UnderControl.Util.Util.inRange(getSIN_Joystick(), -JoystickConstants.DEADBAND,
            JoystickConstants.DEADBAND);
  }

}
