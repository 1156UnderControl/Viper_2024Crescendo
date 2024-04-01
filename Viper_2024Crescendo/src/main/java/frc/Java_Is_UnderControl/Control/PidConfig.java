package frc.Java_Is_UnderControl.Control;

public class PidConfig {

  public final double kP;

  public final double kI;

  public final double kD;

  public final double kF;

  public final double iZone;

  public PidConfig(double kP, double kI, double kD, double kF, double iZone) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.iZone = iZone;
  }

  public PidConfig(double kP, double kI, double kD, double kF) {
    this(kP, kI, kD, kF, 0);
  }

  public PidConfig(double kP, double kI, double kD) {
    this(kP, kI, kD, 0);
  }
}
