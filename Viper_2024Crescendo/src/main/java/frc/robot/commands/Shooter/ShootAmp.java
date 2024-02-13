package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class ShootAmp extends Command {
  private RobotSubsystems robot;

  public ShootAmp(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.shooter.prepareAmpShot();
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.atSetpoint();
  }
}
