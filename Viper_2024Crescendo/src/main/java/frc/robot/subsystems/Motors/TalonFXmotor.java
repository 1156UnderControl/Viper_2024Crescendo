package frc.robot.subsystems.Motors;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.Java_Is_UnderControl.Driver.Phoenix6Util;

public class TalonFXmotor implements Motor {
  private TalonFX motor;

  private final MotionMagicVoltage m_angleVoltageSetter = new MotionMagicVoltage(0);

  private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0);

  private TalonFXConfiguration configuration = new TalonFXConfiguration();

  private Supplier<Double> velocity;

  private Supplier<Double> position;

  private boolean factoryDefaultOcurred = false;

  private DoubleLogEntry appliedOutputLog;

  private DoubleLogEntry targetOutputLog;

  private DoubleLogEntry currentLog;

  private DoubleLogEntry positionLog;

  private DoubleLogEntry velocityLog;

  private DoubleLogEntry temperatureLog;

  private IntegerLogEntry faultsLog;

  private DoubleLogEntry targetPositionLog;

  private DoubleLogEntry targetSpeedLog;

  public TalonFXmotor(int id) {
    motor = new TalonFX(id);
    this.factoryDefault();
    this.clearStickyFaults();
    this.setCurrentLimit(80);
    this.setupLogs(id);
    this.updateLogs();
    this.factoryDefault();
    this.clearStickyFaults();
  }

  private void setupLogs(int motorId) {
    this.appliedOutputLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/appliedOutput");
    this.targetOutputLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/targetOutput");
    this.currentLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/current");
    this.positionLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/position");
    this.velocityLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/velocity");
    this.temperatureLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/temperature");
    this.faultsLog = new IntegerLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/faults");
    this.targetPositionLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/targetPosition");
    this.targetSpeedLog = new DoubleLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/targetSpeed");
    StringLogEntry descriptionLog = new StringLogEntry(DataLogManager.getLog(),
        "/motors/" + motorId + "/Description");
    descriptionLog.append(this.motor.getDescription());
    BooleanLogEntry isInvertedLog = new BooleanLogEntry(DataLogManager.getLog(), "/motors/" + motorId + "/isInverted");
    isInvertedLog.append(this.motor.getInverted());
  }

  @Override
  public void factoryDefault() {
    if (!factoryDefaultOcurred) {
      configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      Phoenix6Util.checkErrorAndRetry(() -> motor.getConfigurator().apply(configuration, 100));
      m_angleVoltageSetter.UpdateFreqHz = 0;
      m_velocityVoltageSetter.UpdateFreqHz = 0;
    }
    factoryDefaultOcurred = true;
  }

  @Override
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1       Applied Motor Output, Fault Information, Limit Switch
   *                         Information
   * @param CANStatus2       Selected Sensor Position (PID 0), Selected Sensor
   *                         Velocity (PID 0), Brushed Supply Current
   *                         Measurement, Sticky Fault Information
   * @param CANStatus3       Quadrature Information
   * @param CANStatus4       Analog Input, Supply Battery Voltage, Controller
   *                         Temperature
   * @param CANStatus8       Pulse Width Information
   * @param CANStatus10      Motion Profiling/Motion Magic Information
   * @param CANStatus12      Selected Sensor Position (Aux PID 1), Selected Sensor
   *                         Velocity (Aux PID 1)
   * @param CANStatus13      PID0 (Primary PID) Information
   * @param CANStatus14      PID1 (Auxiliary PID) Information
   * @param CANStatus21      Integrated Sensor Position (Talon FX), Integrated
   *                         Sensor Velocity (Talon FX)
   * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless
   *                         Stator Current Measurement
   */
  public void configureCANStatusFrames(
      int CANStatus1,
      int CANStatus2,
      int CANStatus3,
      int CANStatus4,
      int CANStatus8,
      int CANStatus10,
      int CANStatus12,
      int CANStatus13,
      int CANStatus14,
      int CANStatus21,
      int CANStatusCurrent) {
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
    // CANStatus2);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
    // CANStatus3);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
    // CANStatus4);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
    // CANStatus8);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets,
    // CANStatus10);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
    // CANStatus12);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
    // CANStatus13);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1,
    // CANStatus14);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated,
    // CANStatus21);
    // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current,
    // CANStatusCurrent);

    // TODO: Configure Status Frame 2 thru 21 if necessary
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F, double Izone) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'configurePIDF'");
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'configurePIDF'");
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'configurePIDWrapping'");
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setMotorBrake'");
  }

  @Override
  public void setInverted(boolean inverted) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
  }

  @Override
  public void setInvertedEncoder(boolean inverted) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setInvertedEncoder'");
  }

  @Override
  public void burnFlash() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'burnFlash'");
  }

  @Override
  public void set(double percentOutput) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'set'");
  }

  @Override
  public void setPositionReference(double position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPositionReference'");
  }

  @Override
  public void setPositionReferenceArbFF(double position, double feedforward) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPositionReferenceArbFF'");
  }

  @Override
  public void setVelocityReference(double velocity, double feedforward) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVelocityReference'");
  }

  @Override
  public double getVoltage() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVoltage'");
  }

  @Override
  public void setVoltage(double voltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }

  @Override
  public double getAppliedOutput() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAppliedOutput'");
  }

  @Override
  public double getVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
  }

  @Override
  public double getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }

  @Override
  public void setPositionFactor(double factor) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPositionFactor'");
  }

  @Override
  public void setVelocityFactor(double factor) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVelocityFactor'");
  }

  @Override
  public void setPosition(double position) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setVoltageCompensation'");
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimit'");
  }

  @Override
  public void setLoopRampRate(double rampRate) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setLoopRampRate'");
  }

  @Override
  public Object getMotor() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getMotor'");
  }

  @Override
  public void updateLogs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateLogs'");
  }

}
