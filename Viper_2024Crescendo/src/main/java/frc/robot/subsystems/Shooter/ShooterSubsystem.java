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
import frc.robot.subsystems.Sensors.DigitalInput;
import frc.robot.subsystems.Sensors.InfraRed;

public class ShooterSubsystem extends SubsystemBase implements IShooter {

  private static ShooterSubsystem shooterInstance = null;

  private boolean hasNote;

  private double kDt = 0.02;

  private double goal = 0;

  private Motor upperFlywheelMotor = new SparkMAXMotor(ShooterConstants.ID_shooterUp);

  private Motor lowerFlywheelMotor = new SparkMAXMotor(ShooterConstants.ID_shooterDown);

  private Motor feederMotor = new SparkMAXMotor(ShooterConstants.ID_feeder);

  private Motor pivotMotor = new SparkMAXMotor(ShooterConstants.ID_pivotShooter, true);

  private DigitalInput noteSensor = new InfraRed(ShooterConstants.port_IR, false);

  SimpleMotorFeedforward pivotFeedforward = new SimpleMotorFeedforward(0.63, 0.04);

  TrapezoidProfile pivotMotionProfile = new TrapezoidProfile(new Constraints(277, 1600));

  private TrapezoidProfile.State pivotCurrentSetpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();

  public static ShooterSubsystem getInstance() {
    if (shooterInstance == null) {
      shooterInstance = new ShooterSubsystem();
    }
    return shooterInstance;
  }

  private ShooterSubsystem() {
    this.hasNote = false;
    this.configureFlywheelMotor(this.upperFlywheelMotor);
    this.configureFlywheelMotor(this.lowerFlywheelMotor);
    this.configureFeederMotor();
    this.configurePivotMotor();
    this.goToSecuredPosition();
    this.initializeSmartDashboardVariables();
  }

  private void configureFlywheelMotor(Motor flywheelMotor) {
    // Using this method for both rollers ensures that
    // they are forced to use the same configuration.
    flywheelMotor.setMotorBrake(false);
    flywheelMotor.setLoopRampRate(0.2);
    flywheelMotor.setVelocityFactor(1.8);
    flywheelMotor.configurePIDF(0, 0, 0, 0);
  }

  private void configureFeederMotor() {
    feederMotor.setMotorBrake(false);
    feederMotor.setLoopRampRate(0.2);
  }

  private void configurePivotMotor() {
    pivotMotor.setMotorBrake(true);
    pivotMotor.setLoopRampRate(0);
    pivotMotor.setVelocityFactor(1 / 0.52);
    pivotMotor.setPositionFactor(264 / 2.32);
    pivotMotor.setInvertedEncoder(true);
    pivotMotor.configurePIDF(0.025, 0, 0, 0);
    pivotMotor.setPosition(0);
  }

  private void initializeSmartDashboardVariables() {
    SmartDashboard.putNumber("P Shooter", 0);
    SmartDashboard.putNumber("I Shooter", 0);
    SmartDashboard.putNumber("D Shooter", 0);
    SmartDashboard.putNumber("I zone Shooter", 0);
    SmartDashboard.putBoolean("setPID", false);
  }

  @Override
  public void periodic() {
    this.updatePivotPidFromDashboard();
    this.pivotMotor.setPositionReferenceArbFF(profilePosition(goal), getFeedForwardProfiled());
    this.updateSmartDashboard();
  }

  private void updatePivotPidFromDashboard() {
    if (SmartDashboard.getBoolean("setPID", false)) {
      double p = SmartDashboard.getNumber("P Shooter", 0);
      double i = SmartDashboard.getNumber("I Shooter", 0);
      double d = 0;
      double iZone = SmartDashboard.getNumber("I Zone", 0);
      this.pivotMotor.configurePIDF(p, i, d, iZone);
    }
  }

  private void updateSmartDashboard() {
    SmartDashboard.putBoolean("IR", this.noteSensor.getBoolean());
    SmartDashboard.putNumber("Position Pivot", this.pivotMotor.getPosition());
    SmartDashboard.putNumber("Velocity Pivot", this.pivotMotor.getVelocity());
    SmartDashboard.putNumber("Output Pivot Shooter", this.pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Velocity ShooterUp", this.upperFlywheelMotor.getVelocity());
    SmartDashboard.putNumber("Velocity ShooterDown", this.lowerFlywheelMotor.getVelocity());
    SmartDashboard.putNumber("Setpoint Shooter", this.pivotCurrentSetpoint.position);
  }

  @Override
  public void intake() {
    this.goToIntakePosition();
    if (this.noteSensor.getBoolean()) {
      this.hasNote = true;
    }
    this.feederMotor.set(this.hasNote ? 0 : ShooterConstants.setpoints.VELOCITY_COLLECT);
  }

  @Override
  public void stopIntaking() {
    this.feederMotor.set(0);
  }

  @Override
  public void goToIntakePosition() {
    this.goal = 46;
  }

  @Override
  public boolean hasNote() {
    return this.hasNote;
  }

  @Override
  public void prepareAmpShot() {
    this.lowerFlywheelMotor.set(-0.8);
    this.upperFlywheelMotor.set(-0.8);
    this.goal = 125;
  }

  @Override
  public void prepareSpeakerShot() {

  }

  @Override
  public void shoot() {
    this.feederMotor.set(0.8);
    this.hasNote = false;
  }

  @Override
  public void goToSecuredPosition() {
    this.lowerFlywheelMotor.set(0);
    this.upperFlywheelMotor.set(0);
    this.feederMotor.set(0);
    goal = 0;
  }

  @Override
  public boolean hasShooted() {
    return !noteSensor.getBoolean();
  }

  @Override
  public boolean isSecureForIntake() {
    return false;
  }

  @Override
  public boolean isAtSetpoint() {
    double minPosition = goal - ShooterConstants.pivotGoalDeadband;
    double maxPosition = goal + ShooterConstants.pivotGoalDeadband;
    return Util.inRange(this.pivotMotor.getPosition(), minPosition, maxPosition);
  }

  @Override
  public void setManualPivotPosition(double goal) {
    this.goal = goal;
    this.periodic();
  }

  private double profilePosition(double goal) {
    this.pivotGoal = new State(goal, 0);
    this.pivotCurrentSetpoint = pivotMotionProfile.calculate(kDt, this.pivotCurrentSetpoint, this.pivotGoal);
    if (this.pivotCurrentSetpoint.position > ShooterConstants.setpoints.LIMIT_MAX) {
      return ShooterConstants.setpoints.LIMIT_MAX;
    }
    if (this.pivotCurrentSetpoint.position < ShooterConstants.setpoints.LIMIT_MIN) {
      return ShooterConstants.setpoints.LIMIT_MIN;
    }
    return this.pivotCurrentSetpoint.position;
  }

  private double getFeedForwardProfiled() {
    return pivotFeedforward.calculate(pivotCurrentSetpoint.velocity / 12);
  }
}
