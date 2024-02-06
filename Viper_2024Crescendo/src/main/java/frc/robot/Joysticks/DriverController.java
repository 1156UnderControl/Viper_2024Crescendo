package frc.robot.Joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;

public class DriverController {
  private DriverController() {
  }

  XboxController driverXbox = new XboxController(0);

  public final double Y_Translation = -MathUtil.applyDeadband(driverXbox.getLeftX(), JoystickConstants.DEADBAND),
  public final double X_Translation = -MathUtil.applyDeadband(driverXbox.getLeftY(), JoystickConstants.DEADBAND),
  public final double COS_JoystickAngle = -driverXbox.getRightX();
  public final double SIN_JoystickAngle = -driverXbox.getRightY();
  public final double Throttle = driverXbox.getRightTriggerAxis();
  public final boolean TurboActivate = 0.5 < driverXbox.getLeftTriggerAxis();

  public final boolean notUsingJoystick = frc.Java_Is_UnderControl.Util.Util.inRange(COS_JoystickAngle,
      -JoystickConstants.DEADBAND,
      0.2)
      && frc.Java_Is_UnderControl.Util.Util.inRange(SIN_JoystickAngle, -JoystickConstants.DEADBAND,
          JoystickConstants.DEADBAND);

}
