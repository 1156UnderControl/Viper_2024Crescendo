package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;

public class IntakeSubsystem extends SubsystemBase implements IIntake {
  private static IntakeSubsystem intakeInstance = null;

  private double kDt = 0.02;

  private double goal = IntakeConstants.setpoints.ANGLE_SECURED;

  private Motor rollerMotor = new SparkMAXMotor(IntakeConstants.ID_intake);

  private Motor pivotMotor = new SparkMAXMotor(IntakeConstants.ID_pivotIntake, true);

  private SimpleMotorFeedforward pivotFeedforward = new SimpleMotorFeedforward(IntakeConstants.tunning_values.KS,
      IntakeConstants.tunning_values.KV);

  private TrapezoidProfile pivotMotionProfile = new TrapezoidProfile(
      new Constraints(IntakeConstants.tunning_values.MAX_VELOCITY,
          IntakeConstants.tunning_values.MAX_ACCELERATION));

  private TrapezoidProfile.State pivotCurrentSetpoint = new TrapezoidProfile.State();

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();

  private CustomDoubleLogger pivotGoalLog = new CustomDoubleLogger("/IntakeSubsystem/PivotGoal");
  private CustomBooleanLogger isAtSetpointLog = new CustomBooleanLogger("/IntakeSubsystem/IsAtSetpoint");
  private String state = "SAFE_POSITION";
  private CustomStringLogger stateLog = new CustomStringLogger("/IntakeSubsystem/State");

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
    this.updateLogs();
  }

  private void configurePivotMotor() {
    this.pivotMotor.setInverted(true);
    this.pivotMotor.setMotorBrake(true);
    this.pivotMotor.setLoopRampRate(0);
    this.pivotMotor.setPositionFactor(360);
    this.pivotMotor.setVelocityFactor(6);
    this.pivotMotor.configurePIDF(IntakeConstants.tunning_values.PID.P, IntakeConstants.tunning_values.PID.I,
        IntakeConstants.tunning_values.PID.D, IntakeConstants.tunning_values.PID.F);
    this.pivotMotor.setPosition(IntakeConstants.setpoints.ANGLE_SECURED);
  }

  @Override
  public void setBrakePivot() {
    this.pivotMotor.setMotorBrake(true);
  }

  @Override
  public void setCoastPivot() {
    this.pivotMotor.setMotorBrake(false);
  }

  private void configureRollerMotor() {
    this.rollerMotor.setInverted(true);
    this.rollerMotor.setMotorBrake(false);
    this.rollerMotor.setLoopRampRate(0.2);
  }

  @Override
  public void periodic() {
    this.pivotMotor.setPositionReferenceArbFF(profilePosition(goal), getFeedForwardProfiled());
    this.updateLogs();
  }

  @Override
  public void intake() {
    this.goToIntakePosition();
    this.rollerMotor.set(IntakeConstants.setpoints.SPEED_INTAKE);
  }

  @Override
  public void intakeFromHPposition() {
    this.goToIntakePosition();
    this.rollerMotor.set(0);
  }

  @Override
  public void stopIntaking() {
    this.rollerMotor.set(0);
  }

  @Override
  public void goToIntakePosition() {
    this.goal = IntakeConstants.setpoints.ANGLE_INTAKE;
    this.state = "INTAKING";
  }

  @Override
  public void shootingPosition() {
    this.goal = 0;
    this.state = "SHOOTING_POSITION";
  }

  @Override
  public void goToSecuredPosition() {
    this.rollerMotor.set(0);
    this.goal = IntakeConstants.setpoints.ANGLE_SECURED;
    this.state = "SAFE_POSITION";
  }

  @Override
  public boolean isSecureForShooter() {
    return this.pivotMotor.getPosition() < IntakeConstants.ANGLE_PASSED_FOR_SHOOTER_MOVE;
  }

  @Override
  public void expell() {
    this.rollerMotor.set(IntakeConstants.setpoints.SPEED_EXPELL);
    this.state = "EXPELL";
  }

  @Override
  public boolean isAtSetpoint() {
    double minPosition = this.goal - IntakeConstants.pivotDeadband;
    double maxPosition = this.goal + IntakeConstants.pivotDeadband;
    return Util.inRange(this.pivotMotor.getPosition(), minPosition, maxPosition);
  }

  @Override
  public void setManualPivotPosition(double goal) {
    this.goal = goal;
    this.state = "MANUAL";
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

  private void updateLogs() {
    this.pivotGoalLog.append(this.goal);
    this.isAtSetpointLog.append(this.isAtSetpoint());
    this.stateLog.append(this.state);
    this.pivotMotor.updateLogs();
    this.rollerMotor.updateLogs();
  }

}
