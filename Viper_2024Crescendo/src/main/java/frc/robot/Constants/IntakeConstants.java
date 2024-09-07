package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {
  public static final Pose2d intakeRollerPositionRelativeToRobotCenter = new Pose2d(0.46, 0, new Rotation2d());

  public static final int ID_pivotIntake = 9; // Adicionar o correto
  public static final int ID_intake = 12; // Adicionar o correto

  public static final int port_IR = 2;

  public static final double POSITION_FACTOR = 360;
  public static final double VELOCITY_FACTOR = 6;

  public static final double pivotDeadband = 1;

  public static final double ANGLE_PASSED_FOR_SHOOTER_MOVE = 100;

  public class tunning_values {
    public static final double KS = 0.36;
    public static final double KV = 0.02;
    public static final double MAX_VELOCITY = 400;
    public static final double MAX_ACCELERATION = 2500;

    public class PID {
      public static final double P = 0.018;
      public static final double I = 0;
      public static final double D = 0;
      public static final double F = 0;

    }

  }

  public class setpoints {

    public static final double SPEED_INTAKE = 1;
    public static final double SPEED_EXPELL = -1;

    public static final double LIMIT_MAX = 140;
    public static final double LIMIT_MIN = 0;

    public static final double ANGLE_SECURED = 140;
    public static final double ANGLE_INTAKE = 0;
  }
}
