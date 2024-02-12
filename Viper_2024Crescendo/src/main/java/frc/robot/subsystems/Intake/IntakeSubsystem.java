package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;
import frc.robot.subsystems.Shooter.IShooter;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeSubsystem extends SubsystemBase implements IIntake {
  private static IntakeSubsystem mInstance = null;

  public static IntakeSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new IntakeSubsystem();
    }

    return mInstance;
  }

  private static double kDt = 0.02;

  Motor mIntake = new SparkMAXMotor(IntakeConstants.ID_intake);
  Motor mPivotIntake = new SparkMAXMotor(IntakeConstants.ID_pivotIntake, true);

  IShooter InterfaceShooter = ShooterSubsystem.getInstance();

  SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(0.28, 0.03);
  TrapezoidProfile mProfile = new TrapezoidProfile(new Constraints(337, 2000));
  private TrapezoidProfile.State mSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoal = new TrapezoidProfile.State();

  private IntakeSubsystem() {
    mPivotIntake.setPosition(0);
    mPivotIntake.setInverted(true);

    mPivotIntake.setMotorBrake(true);
    mIntake.setMotorBrake(false);

    mPivotIntake.setLoopRampRate(0.2);
    mIntake.setLoopRampRate(0.2);

    mPivotIntake.setPositionFactor(360);
    mPivotIntake.setVelocityFactor(6);

    mPivotIntake.configurePIDF(0.025, 0, 0, 0);

    SmartDashboard.putNumber("Position Manual Setpoint", 20);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("kv intake", mPivotIntake.getVelocity() / mPivotIntake.getVoltage());
    SmartDashboard.putNumber("Position  intake Pivot", mPivotIntake.getPosition());
    SmartDashboard.putNumber("Velocity intake Pivot", mPivotIntake.getVelocity());
    SmartDashboard.putNumber("Voltage Collect", mIntake.getVoltage());
    SmartDashboard.putNumber("Output Pivot intake", mPivotIntake.getAppliedOutput());
  }

  @Override
  public void intake() {

    mIntake.set(IntakeConstants.intakeSpeed);
  }

  @Override
  public void freewayShooter() {

  }

  @Override
  public void securedPosition() {

    mIntake.setPositionReferenceArbFF(profilePosition(40), 0);

  }

  @Override
  public boolean secureForShooter() {
    return false;
  }

  @Override
  public void expell() {

  }

  @Override
  public void hasCollected() {

  }

  public void tuningState(double goal) {
    mPivotIntake.setPositionReferenceArbFF(profilePosition(goal), getFeedForwardProfiled());
  }

  public void setVoltage(double voltage) {
    mPivotIntake.setVoltage(voltage);
  }

  private double profilePosition(double goal) {
    mGoal = new State(goal, 0);
    mSetpoint = mProfile.calculate(kDt, mSetpoint, mGoal);
    SmartDashboard.putNumber("setpoint intake", mSetpoint.position);
    return mSetpoint.position;
  }

  private double getFeedForwardProfiled() {

    return mFeedforward.calculate(mSetpoint.velocity / 12);
  }

  @Override
  public boolean atSetpoint() {
    return Util.inRange(mPivotIntake.getPosition(), -1, 1);
  }

}
