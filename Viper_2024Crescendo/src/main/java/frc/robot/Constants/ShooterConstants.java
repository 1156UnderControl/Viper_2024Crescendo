package frc.robot.Constants;

public class ShooterConstants {
  public static final int ID_shooterUp = 15;
  public static final int ID_shooterDown = 14;
  public static final int ID_feeder = 16;
  public static final int ID_pivotShooter = 10;

  public static final int port_IR = 1;

  public static final double pivotGoalDeadband = 5;

  public class tunning_values_pivot {
    public static final double KS = 0.63;
    public static final double KV = 0.04;
    public static final double MAX_VELOCITY = 277;
    public static final double MAX_ACCELERATION = 1600;
  }

  public class tunning_values_shooter {
    public static final double KS = 0.63;
    public static final double KV = 0.04;
    public static final double MAX_VELOCITY = 0.63;
    public static final double MAX_ACCELERATION = 0.04;

    public class PID {
      public static final double P = 0.00006;
      public static final double I = 0;
      public static final double D = 0;
      public static final double F = 0.00009;
      public static final double IZone = 1000;
    }
  }

  public class setpoints {
    public static final double LIMIT_MAX = 230;
    public static final double LIMIT_MIN = 0;

    public static final double ANGLE_SECURED = 0;
    public static final double ANGLE_INTAKE = 41;
    public static final double ANGLE_SHOOT_AMP = 125;
    public static final double ANGLE_SHOOT_SPEAKER = 1;

    public static final double VELOCITY_SHOOT_SPEAKER = 8000;
    public static final double VELOCITY_SHOOT_AMP = 7500;
    public static final double VELOCITY_CRUISE_SHOOTER = 6000;
    public static final double VELOCITY_SHOOTER_CORRECT_PASSED_NOTE = -0.1;
    public static final double VELOCITY_FEED_COLLECT = 0.8;
    public static final double VELOCITY_FEED_COLLECT_AUTO = 0.4;
    public static final double VELOCITY_FEED_SHOOT = 0.8;
    public static final double VELOCITY_FEED_CORRECT_PASSED_NOTE = -0.1;
  }
}
