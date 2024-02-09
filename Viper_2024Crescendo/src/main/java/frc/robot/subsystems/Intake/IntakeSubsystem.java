package frc.robot.subsystems.Intake;

import frc.robot.subsystems.Shooter.IShooter;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class IntakeSubsystem implements IIntake {

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
