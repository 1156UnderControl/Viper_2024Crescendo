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

public class IntakeSubsystem extends SubsystemBase implements IIntake {
  private static IntakeSubsystem intakeInstance = null;

  private double kDt = 0.02;

  private double goal = 140;

  private Motor rollerMotor = new SparkMAXMotor(IntakeConstants.ID_intake);

  private Motor pivotMotor = new SparkMAXMotor(IntakeConstants.ID_pivotIntake, true);

  private SimpleMotorFeedforward pivotFeedforward = new SimpleMotorFeedforward(0.28, 0.03);

  private TrapezoidProfile pivotMotionProfile = new TrapezoidProfile(new Constraints(400, 2500));

  private TrapezoidProfile.State pivotCurrentSetpoint = new TrapezoidProfile.State();
  
  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();

  public static IntakeSubsystem getInstance() {
    if (intakeInstance == null) {
      intakeInstance = new IntakeSubsystem();
    }
    return intakeInstance;
  }

  private IntakeSubsystem() {
    this.configurePivotMotor();
    this.configureRollerMotor();
    SmartDashboard.putNumber("Position Manual Setpoint Shooter", 0);
    this.updateSmartDashboard();
  }

  private void configurePivotMotor() {
    this.pivotMotor.setInverted(true);
    this.pivotMotor.setMotorBrake(true);
    this.pivotMotor.setLoopRampRate(0);
    this.pivotMotor.setPositionFactor(360);
    this.pivotMotor.setVelocityFactor(6);
    this.pivotMotor.configurePIDF(0.025, 0, 0, 0);
    this.pivotMotor.setPosition(0);
  }

  private void configureRollerMotor() {
    this.rollerMotor.setInverted(true);
    this.rollerMotor.setMotorBrake(false);
    this.rollerMotor.setLoopRampRate(0.2);
  }

  @Override
  public void periodic() {
    this.pivotMotor.setPositionReferenceArbFF(profilePosition(goal), getFeedForwardProfiled());
    this.updateSmartDashboard();
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Position intake Pivot", pivotMotor.getPosition());
    SmartDashboard.putNumber("Velocity intake Pivot", pivotMotor.getVelocity());
    SmartDashboard.putNumber("Voltage Collect", rollerMotor.getVoltage());
    SmartDashboard.putNumber("Output Pivot intake", pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Setpoint Intake", pivotCurrentSetpoint.position);
  }

  @Override
  public void intake() {
    this.goToIntakePosition();
    this.rollerMotor.set(IntakeConstants.intakeSpeed);
  }

  @Override
  public void stopIntaking() {
    this.rollerMotor.set(0);
  }

  @Override
  public void goToIntakePosition() {
    this.goal = 0;
  }

  @Override
  public void freewayShooter() {

  }

  @Override
  public void goToSecuredPosition() {
    this.goal = 140;
  }

  @Override
  public boolean isSecureForShooter() {
    return this.pivotMotor.getPosition() < 90;
  }

  @Override
  public void expell() {

  }

  @Override
  public void setManualPivotPosition(double goal) {
    this.goal = goal;
    this.periodic();
  }

  private double profilePosition(double goal) {
    this.pivotGoal = new State(goal, 0);
    this.pivotCurrentSetpoint = this.pivotMotionProfile.calculate(kDt, pivotCurrentSetpoint, pivotGoal);
    if (pivotCurrentSetpoint.position > IntakeConstants.setpoints.LIMIT_MAX) {
      return IntakeConstants.setpoints.LIMIT_MAX;
    }
    if (pivotCurrentSetpoint.position < IntakeConstants.setpoints.LIMIT_MIN) {
      return IntakeConstants.setpoints.LIMIT_MIN;
    }
    return this.pivotCurrentSetpoint.position;
  }

  private double getFeedForwardProfiled() {
    return this.pivotFeedforward.calculate(pivotCurrentSetpoint.velocity / 12);
  }

  @Override
  public boolean atSetpoint() {
    return Util.inRange(this.pivotMotor.getPosition(), this.pivotMotor.getPosition() - 1, this.pivotMotor.getPosition() + 1);
  }

}
