package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class CollectToShooter extends Command {
  private RobotSubsystems robot;

  public CollectToShooter(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.shooter.goToIntakePosition();
    this.robot.shooter.intake();
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.hasNote();
  }
}
