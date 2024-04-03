package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;
import frc.robot.subsystems.Motors.VortexMotor;
import frc.robot.subsystems.Sensors.DigitalInput;
import frc.robot.subsystems.Sensors.InfraRed;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class ShooterSubsystem extends SubsystemBase implements IShooter {

  private static ShooterSubsystem shooterInstance = null;

  private SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private boolean hasNote;

  private boolean wasNoteIn;

  private boolean manualValues;

  private double setpointVelocityShooter;

  private double kDt = 0.02;

  private double goal = ShooterConstants.setpoints.ANGLE_SECURED;

  private Motor upperFlywheelMotor = new VortexMotor(ShooterConstants.ID_shooterUp);

  private Motor lowerFlywheelMotor = new VortexMotor(ShooterConstants.ID_shooterDown);

  private Motor feederMotor = new SparkMAXMotor(ShooterConstants.ID_feeder);

  private Motor pivotMotor = new SparkMAXMotor(ShooterConstants.ID_pivotShooter, true);

  private DigitalInput noteSensorOut = new InfraRed(ShooterConstants.port_IR_OUT, false);

  private DigitalInput noteSensorIn = new InfraRed(ShooterConstants.port_IR_IN, false);

  SimpleMotorFeedforward pivotFeedforward = new SimpleMotorFeedforward(ShooterConstants.tunning_values_pivot.KS,
      ShooterConstants.tunning_values_pivot.KV);

  TrapezoidProfile pivotMotionProfile = new TrapezoidProfile(
      new Constraints(ShooterConstants.tunning_values_pivot.MAX_VELOCITY,
          ShooterConstants.tunning_values_pivot.MAX_ACCELERATION));

  private TrapezoidProfile.State pivotCurrentSetpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();

  public static InterpolatingDoubleTreeMap pivotAngleMap = new InterpolatingDoubleTreeMap();

  public static InterpolatingDoubleTreeMap flywheelRPMmap = new InterpolatingDoubleTreeMap();

  private DoubleLogEntry pivotGoalLog = new DoubleLogEntry(DataLogManager.getLog(), "/ShooterSubsystem/PivotGoal");

  private BooleanLogEntry isAtSetpointLog = new BooleanLogEntry(DataLogManager.getLog(),
      "/ShooterSubsystem/IsAtSetpoint");

  private BooleanLogEntry flywheelAtSetpointLog = new BooleanLogEntry(DataLogManager.getLog(),
      "/ShooterSubsystem/FlywheelAtSetpoint");

  private BooleanLogEntry hasNoteLog = new BooleanLogEntry(DataLogManager.getLog(), "/ShooterSubsystem/HasNote");

  private String state = "SAFE_POSITION";

  private StringLogEntry stateLog = new StringLogEntry(DataLogManager.getLog(), "/ShooterSubsystem/State");

  private StabilizeChecker lowerFlywheelStability = new StabilizeChecker();
  private StabilizeChecker upperFlywheelStability = new StabilizeChecker();
  private StabilizeChecker pivotStabilizeChecker = new StabilizeChecker();

  private long numberOfHasNoteChecks = 0;

  private Trigger inSensorTrigger;

  private Trigger outSensorTrigger;

  public static ShooterSubsystem getInstance() {
    if (shooterInstance == null) {
      shooterInstance = new ShooterSubsystem();
    }
    return shooterInstance;
  }

  private ShooterSubsystem() {
    this.hasNote = false;
    this.wasNoteIn = false;
    this.configureFlywheelMotor(this.upperFlywheelMotor);
    this.configureFlywheelMotor(this.lowerFlywheelMotor);
    this.configureFeederMotor();
    this.configurePivotMotor();
    this.goToSecuredPosition();
    this.initializeSmartDashboardVariables();
    this.updateLogs();
    inSensorTrigger = new Trigger(this.noteSensorIn);
    inSensorTrigger.onTrue(new InstantCommand(() -> this.wasNoteIn = this.state == "INTAKING" ? true : this.wasNoteIn));
    outSensorTrigger = new Trigger(this.noteSensorOut);
    outSensorTrigger.onTrue(
        new InstantCommand(() -> this.hasNote = this.state == "INTAKING" && this.wasNoteIn ? true : this.hasNote));
  }

  private void configureFlywheelMotor(Motor flywheelMotor) {
    flywheelMotor.setInverted(true);
    flywheelMotor.setMotorBrake(false);
    flywheelMotor.setLoopRampRate(0.2);
    flywheelMotor.setVelocityFactor(1.8);
    flywheelMotor.configurePIDF(ShooterConstants.tunning_values_shooter.PID.P,
        ShooterConstants.tunning_values_shooter.PID.I,
        ShooterConstants.tunning_values_shooter.PID.D,
        ShooterConstants.tunning_values_shooter.PID.F,
        ShooterConstants.tunning_values_shooter.PID.IZone);
  }

  private void configureFeederMotor() {
    this.feederMotor.setMotorBrake(false);
    this.feederMotor.setLoopRampRate(0.2);
  }

  private void configurePivotMotor() {
    this.pivotMotor.setMotorBrake(true);
    this.pivotMotor.setLoopRampRate(0);
    this.pivotMotor.setVelocityFactor(1 / 0.52);
    this.pivotMotor.setPositionFactor(264 / 2.32);
    this.pivotMotor.setInvertedEncoder(true);
    this.pivotMotor.configurePIDF(0.055, 0.00008, 0, 0, 10);
    this.pivotMotor.setPosition(0);
  }

  private void initializeSmartDashboardVariables() {
    SmartDashboard.putNumber("P Shooter", ShooterConstants.tunning_values_shooter.PID.P);
    SmartDashboard.putNumber("I Shooter", ShooterConstants.tunning_values_shooter.PID.I);
    SmartDashboard.putNumber("D Shooter", ShooterConstants.tunning_values_shooter.PID.D);
    SmartDashboard.putNumber("F Shooter", ShooterConstants.tunning_values_shooter.PID.F);
    SmartDashboard.putNumber("I zone Shooter", ShooterConstants.tunning_values_shooter.PID.IZone);
    SmartDashboard.putNumber("Setpoint Vel Shooter", 0);
    SmartDashboard.putNumber("Setpoint Pos Shoot Pivot", 0);
    SmartDashboard.putBoolean("Manual Values Shooter", false);
  }

  @Override
  public void periodic() {
    this.updateManualValuesFromDashboard();
    this.updateSmartDashboard();
    this.pivotMotor.setPositionReferenceArbFF(profilePosition(goal), getFeedForwardProfiled());
    this.updateLogs();
    this.runNoteCorrection();
  }

  private void runNoteCorrection() {
    if (state == "SAFE_POSITION") {
      correctNoteCollected();
    }
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
        SmartDashboard.getNumber("P Shooter", ShooterConstants.tunning_values_shooter.PID.P),
        SmartDashboard.getNumber("I Shooter", ShooterConstants.tunning_values_shooter.PID.I),
        SmartDashboard.getNumber("D Shooter", ShooterConstants.tunning_values_shooter.PID.D),
        SmartDashboard.getNumber("F Shooter", ShooterConstants.tunning_values_shooter.PID.F),
        SmartDashboard.getNumber("I zone Shooter", ShooterConstants.tunning_values_shooter.PID.IZone));
  }

  private void updateSmartDashboard() {
    SmartDashboard.putBoolean("IR Out", this.noteSensorOut.getBoolean());
    SmartDashboard.putBoolean("IR In", this.noteSensorIn.getBoolean());
    SmartDashboard.putNumber("Position Pivot", this.pivotMotor.getPosition());
    SmartDashboard.putNumber("Velocity Pivot", this.pivotMotor.getVelocity());
    SmartDashboard.putNumber("Output Pivot Shooter", this.pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Velocity ShooterUp", this.upperFlywheelMotor.getVelocity());
    SmartDashboard.putNumber("Velocity ShooterDown", this.lowerFlywheelMotor.getVelocity());
    SmartDashboard.putNumber("Setpoint Shooter", this.goal);
    SmartDashboard.putNumber("Velocity Feeder", this.feederMotor.getVelocity());
    SmartDashboard.putNumber("Output Shooter Flywheel", this.upperFlywheelMotor.getAppliedOutput());
    SmartDashboard.putBoolean("hasNote", this.hasNote);
    SmartDashboard.putBoolean("wasNoteIn", this.wasNoteIn);
    SmartDashboard.putString("Intake State", state);
    SmartDashboard.putBoolean("Shooter At Setpoint", this.isPivotAtSetpoint());
  }

  @Override
  public void intake() {
    this.setVelocityFlywheel(0);
    this.goToIntakePosition();
    this.runNoteDetection();
    this.feederMotor.set(this.hasNote ? 0 : ShooterConstants.setpoints.VELOCITY_FEED_COLLECT);
    this.state = "INTAKING";
  }

  @Override
  public void intakeFromHP() {
    this.goToIntakePositionHP();
    this.runNoteDetection();
    this.setVelocityFlywheel(this.hasNote ? 0 : ShooterConstants.setpoints.VELOCITY_SHOOTER_COLLECT_HP);
    this.state = "INTAKING_HP";
  }

  public void intakeAuto() {
    this.goToIntakePosition();
    this.runNoteDetection();
    this.feederMotor.set(this.hasNote ? 0 : ShooterConstants.setpoints.VELOCITY_FEED_COLLECT_AUTO);
    this.state = "INTAKING";
  }

  private void runNoteDetection() {
    this.numberOfHasNoteChecks++;
    SmartDashboard.putBoolean("IR Out", this.noteSensorOut.getBoolean());
    SmartDashboard.putBoolean("IR In", this.noteSensorIn.getBoolean());
    SmartDashboard.putNumber("Number of Note Checks", this.numberOfHasNoteChecks);
    if (noteSensorIn.getBoolean()) {
      this.wasNoteIn = true;
    }
    if (this.wasNoteIn && noteSensorOut.getBoolean()) {
      this.hasNote = true;
      this.feederMotor.set(0);
    }
  }

  @Override
  public void stopIntaking() {
    this.feederMotor.set(0);
  }

  @Override
  public void goToIntakePosition() {
    this.goal = ShooterConstants.setpoints.ANGLE_INTAKE;
  }

  public void goToIntakePositionHP() {
    this.goal = 103;
  }

  @Override
  public boolean hasNote() {
    return this.hasNote;
  }

  @Override
  public void setHasNoteFalse() {
    hasNote = false;
  }

  @Override
  public void setHasNoteTrue() {
    hasNote = true;
  }

  @Override
  public void prepareAmpShot() {
    this.feederMotor.set(0);
    setVelocityFlywheel(ShooterConstants.setpoints.VELOCITY_SHOOT_AMP);
    this.goal = ShooterConstants.setpoints.ANGLE_SHOOT_AMP;
    this.state = "PREPARING_AMP_SHOT";
  }

  @Override
  public void prepareSpeakerShot() {
    if (noteSensorOut.getBoolean() || noteSensorIn.getBoolean()) {
      this.feederMotor.set(0);
      this.setVelocityFlywheel(verifyManualValue(calculateGoalFlywheelSetpointSpeaker(),
          "Setpoint Vel Shooter"));
    } else {
      this.lowerFlywheelMotor.set(ShooterConstants.setpoints.VELOCITY_SHOOTER_CORRECT_PASSED_NOTE);
      this.upperFlywheelMotor.set(ShooterConstants.setpoints.VELOCITY_SHOOTER_CORRECT_PASSED_NOTE);
      this.feederMotor.set(ShooterConstants.setpoints.VELOCITY_FEED_CORRECT_PASSED_NOTE);
    }
    goal = verifyManualValue(calculateGoalPivotSetpointSpeaker(), "Setpoint Pos Shoot Pivot");
    this.state = "PREPARING_SPEAKER_SHOT";
  }

  @Override
  public void prepareShootOverStage() {
    this.feederMotor.set(0);
    if (noteSensorOut.getBoolean() || noteSensorIn.getBoolean()) {
      this.feederMotor.set(0);
      setVelocityFlywheel(ShooterConstants.setpoints.VELOCITY_SHOOT_OVER_STAGE);
    } else {
      this.lowerFlywheelMotor.set(ShooterConstants.setpoints.VELOCITY_SHOOTER_CORRECT_PASSED_NOTE);
      this.upperFlywheelMotor.set(ShooterConstants.setpoints.VELOCITY_SHOOTER_CORRECT_PASSED_NOTE);
      this.feederMotor.set(-0.05);
    }
    this.goal = ShooterConstants.setpoints.ANGLE_SHOOT_OVER_STAGE;
    this.state = "PREPARING_SHOT_OVER_STAGE";
  }

  @Override
  public void shoot() {
    this.feederMotor.set(ShooterConstants.setpoints.VELOCITY_FEED_SHOOT);
    this.hasNote = false;
    this.wasNoteIn = false;
    this.state = "SHOOTING";
  }

  @Override
  public void cruise() {
    setVelocityFlywheel(ShooterConstants.setpoints.VELOCITY_CRUISE_SHOOTER);
  }

  @Override
  public void expell() {
    setVelocityFlywheel(ShooterConstants.setpoints.VELOCITY_CRUISE_SHOOTER);
    feederMotor.set(1);
  }

  @Override
  public void goToSecuredPosition() {
    correctNoteCollected();
    this.goal = ShooterConstants.setpoints.ANGLE_SECURED;
    this.state = "SAFE_POSITION";
  }

  private void correctNoteCollected() {
    if (this.hasNote) {
      if (noteSensorOut.getBoolean() || noteSensorIn.getBoolean()) {
        this.lowerFlywheelMotor.set(0);
        this.upperFlywheelMotor.set(0);
        this.feederMotor.set(0);
      } else {
        this.lowerFlywheelMotor.set(ShooterConstants.setpoints.VELOCITY_SHOOTER_CORRECT_PASSED_NOTE);
        this.upperFlywheelMotor.set(ShooterConstants.setpoints.VELOCITY_SHOOTER_CORRECT_PASSED_NOTE);
        this.feederMotor.set(ShooterConstants.setpoints.VELOCITY_FEED_CORRECT_PASSED_NOTE);
      }
    } else {
      this.lowerFlywheelMotor.set(0);
      this.upperFlywheelMotor.set(0);
      this.feederMotor.set(0);
    }
  }

  @Override
  public void stopFeeder() {
    this.feederMotor.set(0);
  }

  @Override
  public boolean hasShooted() {
    return !noteSensorOut.getBoolean();
  }

  @Override
  public boolean getNoteSensor() {
    return noteSensorOut.getBoolean();
  }

  public boolean getAnyNoteSensor() {
    return noteSensorOut.getBoolean() || noteSensorIn.getBoolean();
  }

  @Override
  public boolean isSecureForIntake() {
    return this.pivotMotor.getPosition() < 40;
  }

  @Override
  public boolean isPivotAtSetpoint() {
    boolean atSetpoint = this.pivotStabilizeChecker.isStableInRange(
        this.pivotMotor.getPosition(),
        this.goal,
        ShooterConstants.pivotGoalDeadband);
    return atSetpoint;
  }

  @Override
  public boolean isPivotAtSetpointForAutoShoot() {
    boolean atSetpoint = this.pivotStabilizeChecker.isStableInRange(
        this.pivotMotor.getPosition(),
        this.goal,
        ShooterConstants.pivotGoalAutoShootDeadband);
    return atSetpoint;
  }

  @Override
  public void setManualPivotPosition(double goal) {
    setVelocityFlywheel(ShooterConstants.setpoints.VELOCITY_SHOOT_SPEAKER);
    if (hasNote && !noteSensorOut.getBoolean() && !noteSensorIn.getBoolean()) {
      this.feederMotor.set(ShooterConstants.setpoints.VELOCITY_FEED_CORRECT_PASSED_NOTE);
    } else {
      this.feederMotor.set(0);
    }
    this.goal = goal;
    this.state = "PREPARING_SET_PIVOT";
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

  private double calculateGoalPivotSetpointSpeaker() {
    interpolationTreePivotAngle();
    double setpointInterpolation = pivotAngleMap.get(swerve.distanceFromSpeaker());
    if (setpointInterpolation > ShooterConstants.setpoints.LIMIT_MAX_INTERPOLATION) {
      return ShooterConstants.setpoints.LIMIT_MAX_INTERPOLATION;
    }

    if (setpointInterpolation < ShooterConstants.setpoints.LIMIT_MIN_INTERPOLATION) {
      return ShooterConstants.setpoints.LIMIT_MIN_INTERPOLATION;
    }

    return setpointInterpolation;
  }

  private static void interpolationTreePivotAngle() {
    pivotAngleMap.put(1.1, 47.0);
    pivotAngleMap.put(1.47, 54.0);
    pivotAngleMap.put(1.72, 58.0);
    pivotAngleMap.put(2.12, 62.5);
    pivotAngleMap.put(2.48, 65.0);
    pivotAngleMap.put(2.48, 65.0);
    pivotAngleMap.put(2.84, 67.5);
    pivotAngleMap.put(3.19, 71.5);
    pivotAngleMap.put(3.57, 72.75);
  }

  private double calculateGoalFlywheelSetpointSpeaker() {
    interpolationTreeFlywheelRPM();
    double setpointInterpolation = pivotAngleMap.get(swerve.distanceFromSpeaker());
    if (setpointInterpolation > ShooterConstants.setpoints.LIMIT_MAX_INTERPOLATION_RPM) {
      return ShooterConstants.setpoints.LIMIT_MAX_INTERPOLATION_RPM;
    }

    if (setpointInterpolation < ShooterConstants.setpoints.LIMIT_MIN_INTERPOLATION_RPM) {
      return ShooterConstants.setpoints.LIMIT_MIN_INTERPOLATION_RPM;
    }

    return setpointInterpolation;
  }

  private static void interpolationTreeFlywheelRPM() {
    flywheelRPMmap.put(1.1, 5000.0);
    flywheelRPMmap.put(1.47, 6000.0);
    flywheelRPMmap.put(1.72, 7000.0);
    flywheelRPMmap.put(2.12, 8000.0);
    flywheelRPMmap.put(2.48, 8000.0);
    flywheelRPMmap.put(2.48, 8000.0);
    flywheelRPMmap.put(2.84, 8000.0);
    flywheelRPMmap.put(3.19, 8500.0);
    flywheelRPMmap.put(3.57, 8500.0);
  }

  private void setVelocityFlywheel(double velocity) {
    this.lowerFlywheelMotor.setVelocityReference(velocity, 0);
    this.upperFlywheelMotor.setVelocityReference(velocity, 0);
    this.setpointVelocityShooter = velocity;
  }

  public boolean isFlywheelAtSetpoint() {
    return lowerFlywheelStability.isStableInRange(this.lowerFlywheelMotor.getVelocity(),
        setpointVelocityShooter,
        50)
        && upperFlywheelStability.isStableInRange(this.upperFlywheelMotor.getVelocity(),
            setpointVelocityShooter,
            50);
  }

  private double verifyManualValue(double value, String nameSmartDashboard) {
    return manualValues ? SmartDashboard.getNumber(nameSmartDashboard, value) : value;
  }

  private void updateLogs() {
    this.pivotGoalLog.append(this.goal);
    this.isAtSetpointLog.append(this.isPivotAtSetpoint());
    this.flywheelAtSetpointLog.append(isFlywheelAtSetpoint());
    this.hasNoteLog.append(this.hasNote);
    this.stateLog.append(this.state);
    this.pivotMotor.updateLogs();
    this.feederMotor.updateLogs();
    this.lowerFlywheelMotor.updateLogs();
    this.upperFlywheelMotor.updateLogs();
  }
}
