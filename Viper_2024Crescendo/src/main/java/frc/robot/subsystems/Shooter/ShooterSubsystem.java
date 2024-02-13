package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;
import frc.robot.subsystems.Sensors.InfraRed;

public class ShooterSubsystem extends SubsystemBase implements IShooter {

  private static ShooterSubsystem mInstance = null;

  public static ShooterSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ShooterSubsystem();
    }

    return mInstance;
  }

  private static double kDt = 0.02;

  private static double goal = 0;

  Motor mShooterUp = new SparkMAXMotor(ShooterConstants.ID_shooterUp, false);
  Motor mShooterDown = new SparkMAXMotor(ShooterConstants.ID_shooterDown, false);
  Motor mFeeder = new SparkMAXMotor(ShooterConstants.ID_feeder, false);
  Motor mShooterPivot = new SparkMAXMotor(ShooterConstants.ID_pivotShooter, true);

  frc.robot.subsystems.Sensors.DigitalInput IR = new InfraRed(ShooterConstants.port_IR, false);

  SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(0.63, 0.04);
  TrapezoidProfile mProfile = new TrapezoidProfile(new Constraints(277, 1600));
  private TrapezoidProfile.State mSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoal = new TrapezoidProfile.State();

  private ShooterSubsystem() {
    ShooterConstants.HAS_NOTE = false;

    SmartDashboard.putNumber("P Shooter", 0);
    SmartDashboard.putNumber("I Shooter", 0);
    SmartDashboard.putNumber("D Shooter", 0);
    SmartDashboard.putNumber("I zone Shooter", 0);
    SmartDashboard.putBoolean("setPID", false);

    mShooterUp.setMotorBrake(false);
    mShooterDown.setMotorBrake(false);
    mFeeder.setMotorBrake(false);
    mShooterPivot.setMotorBrake(true);

    mShooterUp.setLoopRampRate(0.2);
    mShooterDown.setLoopRampRate(0.2);
    mFeeder.setLoopRampRate(0.2);
    mShooterPivot.setLoopRampRate(0);

    mShooterUp.setVelocityFactor(1.8);
    mShooterDown.setVelocityFactor(1.8);
    mShooterPivot.setVelocityFactor(1 / 0.52);
    mShooterPivot.setPositionFactor(264 / 2.32);
    mShooterPivot.setInvertedEncoder(true);

    mShooterPivot.setPosition(0);

    mShooterPivot.configurePIDF(0.025, 0, 0, 0);

    mShooterUp.configurePIDF(0, 0, 0, 0);
    mShooterDown.configurePIDF(0, 0, 0, 0);

    securedPosition();
  }

  @Override
  public void periodic() {
    if (SmartDashboard.getBoolean("setPID", false)) {
      mShooterPivot.configurePIDF(SmartDashboard.getNumber("P Shooter", 0),
          SmartDashboard.getNumber("I Shooter", 0),
          0,
          SmartDashboard.getNumber("I Zone", 0));
    }

    SmartDashboard.putBoolean("IR", IR.getBoolean());
    SmartDashboard.putNumber("Position Pivot", mShooterPivot.getPosition());
    SmartDashboard.putNumber("Velocity Pivot", mShooterPivot.getVelocity());
    SmartDashboard.putNumber("Output Pivot Shooter", mShooterPivot.getAppliedOutput());
    SmartDashboard.putNumber("Velocity ShooterUp", mShooterUp.getVelocity());
    SmartDashboard.putNumber("Velocity ShooterDown", mShooterDown.getVelocity());

    mShooterPivot.setPositionReferenceArbFF(profilePosition(goal), getFeedForwardProfiled());
  }

  @Override
  public void intake() {
    if (IR.getBoolean() && !ShooterConstants.HAS_NOTE) {
      ShooterConstants.HAS_NOTE = true;
    }
    mFeeder.set(ShooterConstants.HAS_NOTE ? 0 : ShooterConstants.setpoints.VELOCITY_COLLECT);
  }

  @Override
  public void stopIntaking() {
    mFeeder.set(0);
  }

  @Override
  public void intakePosition() {
    goal = 46;
  }

  @Override
  public boolean hasNote() {
    return ShooterConstants.HAS_NOTE;
  }

  @Override
  public void prepareAmpShot() {
    mShooterDown.set(-0.8);
    mShooterUp.set(-0.8);
    goal = 125;
  }

  @Override
  public void prepareSpeakerShot() {

  }

  @Override
  public void shoot() {
    mFeeder.set(0.8);
  }

  @Override
  public void securedPosition() {
    mShooterDown.set(0);
    mShooterUp.set(0);
    mFeeder.set(0);
    goal = 0;
  }

  @Override
  public boolean hasShooted() {
    return !IR.getBoolean();
  }

  @Override
  public boolean isSecureForIntake() {
    return false;
  }

  public void tuningState(double goal) {
    mShooterPivot.setPositionReferenceArbFF(profilePosition(goal), getFeedForwardProfiled());
  }

  private double profilePosition(double goal) {
    mGoal = new State(goal, 0);
    mSetpoint = mProfile.calculate(kDt, mSetpoint, mGoal);

    SmartDashboard.putNumber("Setpoint Shooter", mSetpoint.position);

    if (mSetpoint.position > ShooterConstants.setpoints.LIMIT_MAX) {
      return ShooterConstants.setpoints.LIMIT_MAX;
    }

    if (mSetpoint.position < ShooterConstants.setpoints.LIMIT_MIN) {
      return ShooterConstants.setpoints.LIMIT_MIN;
    }

    return mSetpoint.position;
  }

  public void setVoltage(double voltage) {
    mShooterPivot.setVoltage(voltage);
  }

  private double getFeedForwardProfiled() {

    return mFeedforward.calculate(mSetpoint.velocity / 12);
  }

  @Override
  public boolean atSetpoint() {
    double minPosition = goal - ShooterConstants.pivotGoalDeadband;
    double maxPosition = goal + ShooterConstants.pivotGoalDeadband;
    return Util.inRange(mShooterPivot.getPosition(), minPosition, maxPosition);
  }
}
