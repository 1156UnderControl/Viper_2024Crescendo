package frc.robot.subsystems.Motors;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomIntegerLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomStringLogger;

public class VortexMotor implements Motor {
  public CANSparkFlex motor;

  public RelativeEncoder encoder;

  public SparkPIDController pid;

  private boolean factoryDefaultOccurred = false;

  private CustomDoubleLogger appliedOutputLog;

  private CustomDoubleLogger currentLog;

  private CustomDoubleLogger positionLog;

  private CustomDoubleLogger velocityLog;

  private CustomDoubleLogger temperatureLog;

  private CustomIntegerLogger faultsLog;

  private double targetPercentage = Double.NaN;

  private CustomDoubleLogger targetOutputLog;

  private double targetPosition = Double.NaN;

  private CustomDoubleLogger targetPositionLog;

  private double targetSpeed = Double.NaN;

  private CustomDoubleLogger targetSpeedLog;

  private double pidP = Double.NaN;

  private double pidI = Double.NaN;

  private double pidD = Double.NaN;

  private double pidF = Double.NaN;

  private double pidIZone = Double.NaN;

  private double wrappingMinInput = Double.NaN;

  private double wrappingMaxInput = Double.NaN;

  private double encoderPositionFactor = Double.NaN;

  private double encoderVelocityFactor = Double.NaN;

  private double nominalVoltage = Double.NaN;

  private double currentLimit = Double.NaN;

  private double rampRate = Double.NaN;

  public VortexMotor(int motorID) {
    this.motor = new CANSparkFlex(motorID, MotorType.kBrushless);
    factoryDefault();
    clearStickyFaults();
    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    pid.setFeedbackDevice(encoder);
    this.setupLogs(motorID, false);
    this.updateLogs();
  }

  private void setupLogs(int motorId, boolean isAlternateEncoder) {
    this.appliedOutputLog = new CustomDoubleLogger("/motors/" + motorId + "/appliedOutput");
    this.currentLog = new CustomDoubleLogger("/motors/" + motorId + "/current");
    this.positionLog = new CustomDoubleLogger("/motors/" + motorId + "/position");
    this.velocityLog = new CustomDoubleLogger("/motors/" + motorId + "/velocity");
    this.temperatureLog = new CustomDoubleLogger("/motors/" + motorId + "/temperature");
    this.faultsLog = new CustomIntegerLogger("/motors/" + motorId + "/faults");
    this.targetOutputLog = new CustomDoubleLogger("/motors/" + motorId + "/targetOutput");
    this.targetPositionLog = new CustomDoubleLogger("/motors/" + motorId + "/targetPosition");
    this.targetSpeedLog = new CustomDoubleLogger("/motors/" + motorId + "/targetSpeed");
    CustomStringLogger firmwareVersionLog = new CustomStringLogger("/motors/" + motorId + "/firmwareVersion");
    firmwareVersionLog.append(this.motor.getFirmwareString());
    CustomBooleanLogger isAlternateEncoderLog = new CustomBooleanLogger("/motors/" + motorId + "/isAlternateEncoder");
    isAlternateEncoderLog.append(isAlternateEncoder);
    CustomBooleanLogger isFollowerLog = new CustomBooleanLogger("/motors/" + motorId + "/isFollower");
    isFollowerLog.append(this.motor.isFollower());
    CustomBooleanLogger isInvertedLog = new CustomBooleanLogger("/motors/" + motorId + "/isInverted");
    isInvertedLog.append(this.motor.getInverted());
  }

  @Override
  public void updateLogs() {
    this.appliedOutputLog.append(this.motor.getAppliedOutput());
    this.currentLog.append(this.motor.getOutputCurrent());
    this.positionLog.append(this.encoder.getPosition());
    this.velocityLog.append(this.encoder.getVelocity());
    this.temperatureLog.append(this.motor.getMotorTemperature());
    this.faultsLog.append(this.motor.getFaults());
    this.targetOutputLog.append(this.targetPercentage);
    this.targetPositionLog.append(this.targetPosition);
    this.targetSpeedLog.append(this.targetSpeed);
  }

  private void configureSparkFlex(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    if (nominalVoltage != this.nominalVoltage) {
      configureSparkFlex(() -> motor.enableVoltageCompensation(nominalVoltage));
    }
    this.nominalVoltage = nominalVoltage;
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    if (currentLimit != this.currentLimit) {
      configureSparkFlex(() -> motor.setSmartCurrentLimit(currentLimit));
    }
    this.currentLimit = currentLimit;
  }

  @Override
  public void setLoopRampRate(double rampRate) {
    if (rampRate != this.rampRate) {
      configureSparkFlex(() -> motor.setClosedLoopRampRate(rampRate));
      configureSparkFlex(() -> motor.setOpenLoopRampRate(rampRate));
    }
    this.rampRate = rampRate;
  }

  @Override
  public Object getMotor() {
    return motor;
  }

  @Override
  public void factoryDefault() {
    if (!factoryDefaultOccurred) {
      configureSparkFlex(motor::restoreFactoryDefaults);
      factoryDefaultOccurred = true;
    }
  }

  @Override
  public void clearStickyFaults() {
    configureSparkFlex(motor::clearFaults);
  }

  @Override
  public void setPositionFactor(double positionConversionFactor) {
    if (positionConversionFactor != this.encoderPositionFactor) {
      configureSparkFlex(() -> encoder.setPositionConversionFactor(positionConversionFactor));
      configureSparkFlex(() -> encoder.setVelocityConversionFactor(positionConversionFactor / 60));
      configureCANStatusFrames(10, 20, 20, 500, 500);
    }
    this.encoderPositionFactor = positionConversionFactor;
  }

  @Override
  public void setVelocityFactor(double factor) {
    if (factor != this.encoderVelocityFactor) {
      configureSparkFlex(() -> encoder.setVelocityConversionFactor(factor));
      configureCANStatusFrames(10, 20, 20, 500, 500);
    }
    this.encoderVelocityFactor = factor;
  }

  @Override
  public void configurePIDF(double p, double i, double d, double f, double iZone) {
    int pidSlot = 0;
    if (p != this.pidP) {
      configureSparkFlex(() -> pid.setP(p, pidSlot));
    }
    if (i != this.pidI) {
      configureSparkFlex(() -> pid.setI(i, pidSlot));
    }
    if (d != this.pidD) {
      configureSparkFlex(() -> pid.setD(d, pidSlot));
    }
    if (f != this.pidF) {
      configureSparkFlex(() -> pid.setFF(f, pidSlot));
    }
    if (iZone != this.pidIZone) {
      configureSparkFlex(() -> pid.setIZone(iZone, pidSlot));
    }
    this.pidP = p;
    this.pidI = i;
    this.pidD = d;
    this.pidF = f;
    this.pidIZone = iZone;
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F) {
    configurePIDF(P, I, D, F, 0);
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    if (minInput != this.wrappingMinInput || maxInput != this.wrappingMaxInput) {
      configureSparkFlex(() -> pid.setPositionPIDWrappingEnabled(true));
      configureSparkFlex(() -> pid.setPositionPIDWrappingMinInput(minInput));
      configureSparkFlex(() -> pid.setPositionPIDWrappingMaxInput(maxInput));
    }
    this.wrappingMinInput = minInput;
    this.wrappingMaxInput = maxInput;
  }

  /**
   * Set the CAN status frames.
   *
   * CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
   * CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor
   * Current
   * CANStatus2 Motor Position
   * CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog
   * Sensor Position
   * CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
   */
  public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4) {
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0));
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1));
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2));
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3));
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4));
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    configureSparkFlex(() -> motor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast));
  }

  @Override
  public void setInverted(boolean inverted) {
    motor.setInverted(inverted);
  }

  @Override
  public void burnFlash() {
    try {
      Thread.sleep(200);
    } catch (Exception e) {
    }
    configureSparkFlex(() -> motor.burnFlash());
  }

  @Override
  public void set(double percentOutput) {
    if (percentOutput != this.targetPercentage) {
      motor.set(percentOutput);
    }
    this.targetPercentage = percentOutput;
    this.targetPosition = Double.NaN;
    this.targetSpeed = Double.NaN;
    this.updateLogs();
  }

  @Override
  public void setPositionReference(double position) {
    if (position != this.targetPosition) {
      configureSparkFlex(() -> pid.setReference(position, ControlType.kPosition, 0));
    }
    this.targetPercentage = Double.NaN;
    this.targetPosition = position;
    this.targetSpeed = Double.NaN;
    this.updateLogs();
  }

  @Override
  public void setPositionReferenceArbFF(double setpoint, double feedforward) {
    if (setpoint != this.targetPosition) {
      configureSparkFlex(() -> pid.setReference(setpoint, ControlType.kPosition, 0, feedforward));
    }
    this.targetPercentage = Double.NaN;
    this.targetPosition = setpoint;
    this.targetSpeed = Double.NaN;
    this.updateLogs();
  }

  @Override
  public void setVelocityReference(double velocity, double feedforward) {
    if (velocity != this.targetSpeed) {
      configureSparkFlex(() -> pid.setReference(velocity, ControlType.kVelocity, 0, feedforward));
    }
    this.targetPercentage = Double.NaN;
    this.targetPosition = Double.NaN;
    this.targetSpeed = velocity;
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
    return encoder.getVelocity();
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void setPosition(double position) {
    // Here it should not have a check comparing to the previous
    // call because the encoder could have moved, so calling setPosition
    // twice with the same value could be a valid use case.
    configureSparkFlex(() -> encoder.setPosition(position));
  }

  @Override
  public void setInvertedEncoder(boolean inverted) {
    configureSparkFlex(() -> encoder.setInverted(inverted));
  }

  @Override
  public void configureFeedForward(double Kg, double Ks, double Kv) {
    // not implemented
  }

  @Override
  public void setPositionReferenceMotionProfiling(double position, double velocity, double feedforward) {
    // not implemented
  }
}
