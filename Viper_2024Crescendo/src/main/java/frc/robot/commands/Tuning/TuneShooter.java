package frc.robot.commands.Tuning;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class TuneShooter extends Command {
  private ShooterSubsystem shooter;

  public TuneShooter(ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.addRequirements(shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    this.shooter.setManualPivotPosition(SmartDashboard.getNumber("Position Manual Setpoint Shooter", 0), 8000);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
