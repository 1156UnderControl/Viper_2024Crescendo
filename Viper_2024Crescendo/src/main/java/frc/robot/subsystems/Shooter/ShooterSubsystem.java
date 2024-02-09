package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  CANSparkMax mShooterUpSpark = new CANSparkMax(ShooterConstants.ID_shooterUp, MotorType.kBrushless);
  CANSparkMax mShooterDownSpark = new CANSparkMax(ShooterConstants.ID_shooterDown, MotorType.kBrushless);
  CANSparkMax mFeederSpark = new CANSparkMax(ShooterConstants.ID_feeder, MotorType.kBrushless);
  CANSparkMax mShooterPivotSpark = new CANSparkMax(ShooterConstants.ID_pivotShooter, MotorType.kBrushless);

  Motor mShooterUp = new SparkMAXMotor(mShooterUpSpark, false);
  Motor mShooterDown = new SparkMAXMotor(mShooterDownSpark, false);
  Motor mFeeder = new SparkMAXMotor(mFeederSpark, false);
  Motor mShooterPivot = new SparkMAXMotor(mShooterPivotSpark, true);

  frc.robot.subsystems.Sensors.DigitalInput IR = new InfraRed(ShooterConstants.port_IR, false);

  SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(1, 0);
  TrapezoidProfile mProfile = new TrapezoidProfile(new Constraints(5, 10));
  private TrapezoidProfile.State mSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoal = new TrapezoidProfile.State();

  private ShooterSubsystem() {
    mShooterUp.setMotorBrake(false);
    mShooterDown.setMotorBrake(false);
    mFeeder.setMotorBrake(false);
    mShooterPivot.setMotorBrake(true);

    mShooterUp.setLoopRampRate(0.2);
    mShooterDown.setLoopRampRate(0.2);
    mFeeder.setLoopRampRate(0.2);
    mShooterPivot.setLoopRampRate(0.2);
    mShooterPivot.setPosition(0);

    mShooterUp.setVelocityFactor(1.8);
    mShooterDown.setVelocityFactor(1 / 23.33);
    mShooterPivot.setVelocityFactor(1 / 0.52);
    mShooterPivot.setPositionFactor(264 / 2.32);
    mShooterPivot.setInvertedEncoder(true);
    mShooterUp.configurePIDF(0, 0, 0, 0);
    mShooterDown.configurePIDF(0, 0, 0, 0);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Position Pivot", mShooterPivot.getPosition());
    SmartDashboard.putNumber("Velocity Pivot", mShooterPivot.getVelocity());
    SmartDashboard.putNumber("Velocity ShooterUp", mShooterUp.getVelocity());
    SmartDashboard.putNumber("Velocity ShooterDown", mShooterDown.getVelocity());
  }

  @Override
  public void intake() {

  }

  @Override
  public boolean hasNote() {
    return false;
  }

  @Override
  public void prepareAmpShot() {

  }

  @Override
  public void prepareSpeakerShot() {

  }

  @Override
  public void shoot() {

  }

  @Override
  public void securedPosition() {
    mShooterPivot.setPositionReference(profilePosition(10), getFeedForwardProfiled());
  }

  @Override
  public boolean secureForIntake() {
    return false;
  }

  private double profilePosition(double goal) {
    mGoal = new State(goal, 0);
    mSetpoint = mProfile.calculate(kDt, mSetpoint, mGoal);
    return mSetpoint.position;
  }

  private double getFeedForwardProfiled() {

    return mFeedforward.calculate(mSetpoint.velocity / 12);
  }

}
