package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;
import frc.robot.subsystems.Motors.VortexMotor;
import frc.robot.subsystems.Sensors.DigitalInput;
import frc.robot.subsystems.Sensors.InfraRed;

public class ShooterSubsystem extends SubsystemBase implements IShooter {

  private static ShooterSubsystem shooterInstance = null;

  private double maxVelocityFeeder;

  private boolean hasNote;

  private boolean manualValues;

  private double kDt = 0.02;

  private double goal = ShooterConstants.setpoints.ANGLE_SECURED;

  private Motor upperFlywheelMotor = new VortexMotor(ShooterConstants.ID_shooterUp);

  private Motor lowerFlywheelMotor = new VortexMotor(ShooterConstants.ID_shooterDown);

  private Motor feederMotor = new SparkMAXMotor(ShooterConstants.ID_feeder);

  private Motor pivotMotor = new SparkMAXMotor(ShooterConstants.ID_pivotShooter, true);

  private DigitalInput noteSensor = new InfraRed(ShooterConstants.port_IR, false);

  SimpleMotorFeedforward pivotFeedforward = new SimpleMotorFeedforward(ShooterConstants.tunning_values_pivot.KS,
      ShooterConstants.tunning_values_pivot.KV);

  TrapezoidProfile pivotMotionProfile = new TrapezoidProfile(
      new Constraints(ShooterConstants.tunning_values_pivot.MAX_VELOCITY,
          ShooterConstants.tunning_values_pivot.MAX_ACCELERATION));

  private TrapezoidProfile.State pivotCurrentSetpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();

  public static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

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
    flywheelMotor.setInverted(true);
    flywheelMotor.setMotorBrake(false);
    flywheelMotor.setLoopRampRate(0.2);
    flywheelMotor.setVelocityFactor(1.8);
    flywheelMotor.configurePIDF(ShooterConstants.tunning_values_shooter.PID.P,
        ShooterConstants.tunning_values_shooter.PID.I,
        ShooterConstants.tunning_values_shooter.PID.D,
        ShooterConstants.tunning_values_shooter.PID.F);
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
    SmartDashboard.putNumber("F Shooter", 0);
    SmartDashboard.putNumber("I zone Shooter", 0);
    SmartDashboard.putNumber("Setpoint Vel Shooter", 0);
    SmartDashboard.putNumber("Setpoint Pos Shoot Pivot", 0);
    SmartDashboard.putBoolean("Manual Values Shooter", false);
  }

  @Override
  public void periodic() {
    this.updateManualValuesFromDashboard();
    this.updateSmartDashboard();
    this.pivotMotor.setPositionReferenceArbFF(profilePosition(goal), getFeedForwardProfiled());
  }

  private void updateManualValuesFromDashboard() {
    manualValues = SmartDashboard.getBoolean("Manual Values Shooter", false);
    if (manualValues) {
      this.configureFlywheelMotorPIDsmartdashboard(lowerFlywheelMotor);
      this.configureFlywheelMotorPIDsmartdashboard(upperFlywheelMotor);
    }
  }

  private void configureFlywheelMotorPIDsmartdashboard(Motor flywheelMotor) {
    flywheelMotor.configurePIDF(
        SmartDashboard.getNumber("P Shooter", 0),
        SmartDashboard.getNumber("I Shooter", 0),
        SmartDashboard.getNumber("D Shooter", 0),
        SmartDashboard.getNumber("F Shooter", 0),
        SmartDashboard.getNumber("I zone Shooter", 0));
  }

  private void updateSmartDashboard() {
    SmartDashboard.putBoolean("IR", this.noteSensor.getBoolean());
    SmartDashboard.putNumber("Position Pivot", this.pivotMotor.getPosition());
    SmartDashboard.putNumber("Velocity Pivot", this.pivotMotor.getVelocity());
    SmartDashboard.putNumber("Output Pivot Shooter", this.pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Velocity ShooterUp", this.upperFlywheelMotor.getVelocity());
    SmartDashboard.putNumber("Velocity ShooterDown", this.lowerFlywheelMotor.getVelocity());
    SmartDashboard.putNumber("Setpoint Shooter", this.pivotCurrentSetpoint.position);
    SmartDashboard.putNumber("Velocity Feeder", this.feederMotor.getVelocity());
    SmartDashboard.putNumber("Max Velocity Feeder", this.maxVelocityFeeder);
    SmartDashboard.putNumber("Output Shooter Flywheel", this.upperFlywheelMotor.getAppliedOutput());
    SmartDashboard.putBoolean("hasNote", this.hasNote);
    SmartDashboard.putBoolean("Shooter At Setpoint", this.isAtSetpoint());
  }

  @Override
  public void intake() {
    this.goToIntakePosition();
    this.runNoteDetection();
    this.feederMotor.set(this.hasNote ? 0 : ShooterConstants.setpoints.VELOCITY_FEED_COLLECT);
  }

  private void runNoteDetection() {
    double velocityFeeder = this.feederMotor.getVelocity();
    if (velocityFeeder > this.maxVelocityFeeder) {
      this.maxVelocityFeeder = velocityFeeder;
    }
    if (noteSensor.getBoolean()) {
      if (velocityFeeder < this.maxVelocityFeeder - 200) {
        hasNote = true;
      }
    }
  }

  @Override
  public void stopIntaking() {
    this.maxVelocityFeeder = 0;
    this.feederMotor.set(0);
  }

  @Override
  public void goToIntakePosition() {
    this.goal = ShooterConstants.setpoints.ANGLE_INTAKE;
  }

  @Override
  public boolean hasNote() {
    return this.hasNote;
  }

  @Override
  public void prepareAmpShot() {
    this.lowerFlywheelMotor.set(-0.5);
    this.upperFlywheelMotor.set(-0.5);
    this.goal = ShooterConstants.setpoints.ANGLE_SHOOT_AMP;
  }

  @Override
  public void prepareSpeakerShot() {
    setVelocityFlywheel(verifyManualValue(ShooterConstants.setpoints.VELOCITY_SHOOT_SPEAKER,
        "Setpoint Vel Shooter"));
    goal = verifyManualValue(46, "Setpoint Pos Shoot Pivot");
  }

  @Override
  public void shoot() {
    this.feederMotor.set(ShooterConstants.setpoints.VELOCITY_FEED_SHOOT);
    this.hasNote = false;
    this.maxVelocityFeeder = 0;
  }

  @Override
  public void goToSecuredPosition() {
    this.lowerFlywheelMotor.set(0);
    this.upperFlywheelMotor.set(0);
    this.feederMotor
        .set(noteSensor.getBoolean() || !hasNote ? 0 : ShooterConstants.setpoints.VELOCITY_FEED_CORRECT_PASSED_NOTE);
    this.goal = ShooterConstants.setpoints.ANGLE_SECURED;
    this.maxVelocityFeeder = 0;
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
    double minPosition = this.goal - ShooterConstants.pivotGoalDeadband;
    double maxPosition = this.goal + ShooterConstants.pivotGoalDeadband;
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

  private static double calculateGoalSpeaker() {
    interpolationTree();
    return shooterMap.get(LimelightConstants.getDistance());
  }

  private static void interpolationTree() {
    shooterMap.put(null, null);
    shooterMap.put(null, null);
    shooterMap.put(null, null);
    shooterMap.put(null, null);
    shooterMap.put(null, null);
  }
  
  private void setVelocityFlywheel(double velocity) {
    this.lowerFlywheelMotor.setVelocityReference(velocity, 0);
    this.upperFlywheelMotor.setVelocityReference(velocity, 0);
  }

  private double verifyManualValue(double value, String nameSmartDashboard) {
    return manualValues ? SmartDashboard.getNumber(nameSmartDashboard, 0) : value;
  }
}
