package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;
import frc.robot.subsystems.Shooter.IShooter;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeSubsystem implements IIntake {
  private static IntakeSubsystem mInstance = null;

  public static IntakeSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new IntakeSubsystem();
    }

    return mInstance;
  }

  private static double kDt = 0.02;

  CANSparkMax mFeederSpark = new CANSparkMax(ShooterConstants.ID_feeder, MotorType.kBrushless);
  CANSparkMax mShooterPivotSpark = new CANSparkMax(ShooterConstants.ID_pivotShooter, MotorType.kBrushless);

  Motor mShooterUp = new SparkMAXMotor(mShooterUpSpark, false);
  Motor mShooterDown = new SparkMAXMotor(mShooterDownSpark, false);

  IShooter aa = ShooterSubsystem.getInstance();

  private IntakeSubsystem() {
    // conversion factor: position: 360, velocity: 6
  }

  @Override
  public void intake() {

  }

  @Override
  public void freewayShooter() {

  }

  @Override
  public void securedPosition() {

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
}
