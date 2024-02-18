package frc.robot.subsystems.Motors;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class SparkMAXMotor implements Motor {
  private CANSparkMax motor;

  private SparkPIDController pid;

  private RelativeEncoder encoder;

  private Supplier<Double> velocity;

  private Supplier<Double> position;

  private boolean factoryDefaultOcurred = false;

  private DoubleLogEntry appliedOutputLog;

  private DoubleLogEntry currentLog;

  private DoubleLogEntry positionLog;

  private DoubleLogEntry velocityLog;

  private DoubleLogEntry temperatureLog;

  private IntegerLogEntry faultsLog;

  private double targetPosition = 0;

  private DoubleLogEntry targetPositionLog;

  private double targetSpeed = 0;

  private DoubleLogEntry targetSpeedLog;

  public SparkMAXMotor(int motorId) {
    this(motorId, false);
  }

  public SparkMAXMotor(int motorID, boolean usingAlternateEncoder) {
    this.motor = new CANSparkMax(motorID, MotorType.kBrushless);
    this.pid = motor.getPIDController();
    this.factoryDefault();
    this.clearStickyFaults();
    this.setCurrentLimit(80);
    this.setupMotorPid(usingAlternateEncoder);
    this.setupLogs(motorID, usingAlternateEncoder);
    this.updateLogs();
    velocity = this.encoder::getVelocity;
    position = this.encoder::getPosition;
  }

  private void configureSparkMax(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  private void setupMotorPid(boolean usingAlternateEncoder) {
    this.setAlternateEncoder(usingAlternateEncoder);
    this.pid.setFeedbackDevice(encoder);
  }

  private void setAlternateEncoder(boolean usingAlternateEncoder) {
    if (usingAlternateEncoder) {
      this.encoder = motor.getAlternateEncoder(8192);
    } else {
      this.encoder = motor.getEncoder();
    }
  }

  private void setupLogs(int motorId, boolean isAlternateEncoder) {
    this.appliedOutputLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/appliedOutput");
    this.currentLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/current");
    this.positionLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/position");
    this.velocityLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/velocity");
    this.temperatureLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/temperature");
    this.faultsLog = new IntegerLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/faults");
    this.targetPositionLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/targetPosition");
    this.targetSpeedLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/targetSpeed");
    StringLogEntry firmwareVersionLog = new StringLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/firmwareVersion");
    firmwareVersionLog.append(this.motor.getFirmwareString());
    BooleanLogEntry isAlternateEncoderLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isAlternateEncoder");
    isAlternateEncoderLog.append(isAlternateEncoder);
    BooleanLogEntry isFollowerLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isFollower");
    isFollowerLog.append(this.motor.isFollower());
    BooleanLogEntry isInvertedLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isInverted");
    isInvertedLog.append(this.motor.getInverted());
  }

  private void updateLogs() {
    this.appliedOutputLog.append(this.motor.getAppliedOutput());
    this.currentLog.append(this.motor.getOutputCurrent());
    this.positionLog.append(this.encoder.getPosition());
    this.velocityLog.append(this.encoder.getVelocity());
    this.temperatureLog.append(this.motor.getMotorTemperature());
    this.faultsLog.append(this.motor.getFaults());
    this.targetPositionLog.append(this.targetPosition);
    this.targetSpeedLog.append(this.targetSpeed);
  }

  /*
   * CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
   * CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
   * CANStatus2 Motor Position
   * CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor
   * Position
   * CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
   * CANStatus5 Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder
   * Absolute Angle
   * CANStatus6 Duty Cycle Absolute Encoder Velocity, Duty Cycle Absolute Encoder
   * Frequency
   */

  public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus5, int CANStatus6) {
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, CANStatus5));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, CANStatus6));
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    configureSparkMax(() -> motor.enableVoltageCompensation(nominalVoltage));
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    configureSparkMax(() -> motor.setSmartCurrentLimit(currentLimit));
  }

  @Override
  public void setLoopRampRate(double rampRate) {
    configureSparkMax(() -> motor.setClosedLoopRampRate(rampRate));
    configureSparkMax(() -> motor.setOpenLoopRampRate(rampRate));
  }

  @Override
  public Object getMotor() {
    return motor;
  }

  @Override
  public void factoryDefault() {
    if (!factoryDefaultOcurred) {
      configureSparkMax(motor::restoreFactoryDefaults);
    }
    factoryDefaultOcurred = true;
  }

  @Override
  public void clearStickyFaults() {
    configureSparkMax(() -> motor.clearFaults());
  }

  @Override
  public void setPositionFactor(double factor) {
    configureSparkMax(() -> encoder.setPositionConversionFactor(factor));
  }

  @Override
  public void setVelocityFactor(double factor) {
    configureSparkMax(() -> encoder.setVelocityConversionFactor(factor));
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F, double Izone) {
    configureSparkMax(() -> pid.setP(P, 0));
    configureSparkMax(() -> pid.setI(I, 0));
    configureSparkMax(() -> pid.setD(D, 0));
    configureSparkMax(() -> pid.setFF(F, 0));
    configureSparkMax(() -> pid.setIZone(Izone, 0));
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F) {
    configureSparkMax(() -> pid.setP(P, 0));
    configureSparkMax(() -> pid.setI(I, 0));
    configureSparkMax(() -> pid.setD(D, 0));
    configureSparkMax(() -> pid.setFF(F, 0));
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(true));
    configureSparkMax(() -> pid.setPositionPIDWrappingMaxInput(maxInput));
    configureSparkMax(() -> pid.setPositionPIDWrappingMinInput(minInput));
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    configureSparkMax(() -> motor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast));
  }

  @Override
  public void setInverted(boolean inverted) {
    motor.setInverted(inverted);
  }

  @Override
  public void setInvertedEncoder(boolean inverted) {
    encoder.setInverted(inverted);
  }

  @Override
  public void burnFlash() {
    try {
      Thread.sleep(200);
    } catch (Exception e) {
    }
    configureSparkMax(() -> motor.burnFlash());
  }

  @Override
  public void set(double percentOutput) {
    this.targetPosition = Double.NaN;
    this.targetSpeed = Double.NaN;
    motor.set(percentOutput);
    this.updateLogs();
  }

  @Override
  public void setPositionReference(double position) {
    this.targetPosition = position;
    this.targetSpeed = Double.NaN;
    configureSparkMax(() -> pid.setReference(position, ControlType.kPosition, 0));
    this.updateLogs();
  }

  @Override
  public void setPositionReferenceArbFF(double position, double feedforward) {
    this.targetPosition = position;
    this.targetSpeed = Double.NaN;
    configureSparkMax(() -> pid.setReference(position, ControlType.kPosition, 0, feedforward));
    this.updateLogs();
  }

  @Override
  public void setVelocityReference(double velocity, double feedforward) {
    this.targetPosition = Double.NaN;
    this.targetSpeed = velocity;
    configureSparkMax(() -> pid.setReference(velocity, ControlType.kPosition, 0, feedforward));
    this.updateLogs();
  }

  @Override
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  @Override
  public double getVelocity() {
    return velocity.get();
  }

  @Override
  public double getPosition() {
    return position.get();
  }

  @Override
  public void setPosition(double position) {
    configureSparkMax(() -> encoder.setPosition(position));
  }

}
