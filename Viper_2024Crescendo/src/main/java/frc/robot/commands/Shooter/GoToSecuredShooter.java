package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class GoToSecuredShooter extends Command {
  private RobotSubsystems robot;

  public GoToSecuredShooter(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(this.robot);
  }

  @Override
  public void execute() {
    this.robot.shooter.securedPosition();
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.atSetpoint();
  }
}
