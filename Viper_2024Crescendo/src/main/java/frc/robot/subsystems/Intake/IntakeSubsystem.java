package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;
import frc.robot.subsystems.Shooter.IShooter;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Intake.IIntake;

public class IntakeSubsystem implements IIntake {
  private static IntakeSubsystem mInstance = null;

  public static IntakeSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new IntakeSubsystem();
    }

    return mInstance;
  }

  private static double kDt = 0.02;

  CANSparkMax mIntakeSparkMax = new CANSparkMax(IntakeConstants.ID_intake, MotorType.kBrushless);
  CANSparkMax mPivotIntakeSparkMax = new CANSparkMax(IntakeConstants.ID_pivotIntake, MotorType.kBrushless);

  Motor mIntake = new SparkMAXMotor(mIntakeSparkMax, false);
  Motor mPivotIntake = new SparkMAXMotor(mPivotIntakeSparkMax, false);

  IShooter InterfaceShooter = ShooterSubsystem.getInstance();

  private IntakeSubsystem() {
    // conversion factor: position: 360, velocity: 6
  }

  @Override
  public void intake() {

    mIntake.set();

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
