package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class GoToSecuredShooter extends Command {
  private ShooterSubsystem shooter;

  public GoToSecuredShooter(ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.addRequirements(shooter);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("voltage to run pivot", 0);

  }

  @Override
  public void execute() {
    this.shooter.setVoltage(SmartDashboard.getNumber("voltage to run pivot", 0));
  }

  @Override
  public boolean isFinished() {
    return this.shooter.isSecureForIntake();
  }
}
