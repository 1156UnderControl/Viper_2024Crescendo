package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class ReleaseClimber extends Command {
  private RobotSubsystems robot;

  public ReleaseClimber(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.climber.release();
  }

  @Override
  public boolean isFinished() {
    return this.robot.climber.isAtSetpoint();
  }
}
