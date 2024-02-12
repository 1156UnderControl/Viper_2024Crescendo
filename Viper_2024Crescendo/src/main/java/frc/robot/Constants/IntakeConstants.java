package frc.robot.Constants;

public class IntakeConstants {
  public static final int ID_pivotIntake = 9; // Adicionar o correto
  public static final int ID_intake = 12; // Adicionar o correto

  public static final int port_IR = 2;

  public static final double intakeSpeed = 0.8;
  public static final double intakeCollectPosition = 180;
  public static final double intakeSafePosition = 0;

  public static final double pidPivotIntakeP = 0.1;
  public static final double pidPivotIntakeI = 0.1;
  public static final double pidPivotIntakeD = 0;

  public class setpoints {
    public static final double LIMIT_MAX = 150;
    public static final double LIMIT_MIN = 0;

    public static final double SECURED = 1;
  }
}
