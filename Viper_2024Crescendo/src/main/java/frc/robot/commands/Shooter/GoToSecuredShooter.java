package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class GoToSecuredShooter extends Command {
  private ShooterSubsystem shooter;

  public GoToSecuredShooter(ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.addRequirements(shooter);
  }

  @Override
  public void execute() {
    this.shooter.securedPosition();
  }

  @Override
  public boolean isFinished() {
    return this.shooter.atSetpoint();
  }
}
