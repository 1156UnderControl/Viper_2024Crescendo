package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class CollectToShooter extends Command {
  private ShooterSubsystem shooter;

  public CollectToShooter(ShooterSubsystem shooter) {
    this.shooter = shooter;
    this.addRequirements(shooter);
  }

  @Override
  public void execute() {
    this.shooter.intakePosition();
    this.shooter.intake();
  }

  @Override
  public boolean isFinished() {
    return this.shooter.hasNote();
  }
}
