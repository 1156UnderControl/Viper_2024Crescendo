package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class PullClimber extends Command {
  private RobotSubsystems robot;

  public PullClimber(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    if (!this.robot.climber.isNearLimit()) {
      this.robot.climber.climb();
    } else {
      this.robot.climber.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
