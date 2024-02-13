package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class ShootSpeaker extends Command {
  private RobotSubsystems robot;

  public ShootSpeaker(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.shooter.prepareSpeakerShot();
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.isAtSetpoint();
  }
}
