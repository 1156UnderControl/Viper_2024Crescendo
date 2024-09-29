package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

public class SwerveConstants {
  public static final double DEADBAND_ALIGNED_WITH_SPEAKER = 5;
  public static final double DEADBAND_AT_AMP_POSITION_X = 0.11;
  public static final double DEADBAND_AT_AMP_POSITION_Y = 0.1;
  public static final double DEADBAND_ALIGNED_WITH_AMP_X = 12;
  public static final double DEADBAND_ALIGNED_WITH_AMP_Y = 12;
  public static final double ALIGNED_WITH_AMP_GOAL = 7.2;

  public static final double MAX_VEL = Units.feetToMeters(14.5);
  public static final double MAX_ROTATION_VEL = 20;

  public static final double ROTATION_BUTTON_SPEED = 8;

  public static final double MOTION_PROFILE_MAX_VEL = 4;
  public static final double MOTION_PROFILE_MAX_ACCEL = 5;

  public static final double DriveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4),
      6.12, 1);
  public static final double SteeringConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);

  public static final double ROBOT_MASS = 52;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, 0.173), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static boolean ACELL_CORREC = true;
  public static final double WHEEL_LOCK_TIME = 10;
}
