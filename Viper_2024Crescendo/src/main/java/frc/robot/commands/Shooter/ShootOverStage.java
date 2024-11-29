package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class ShootOverStage extends Command {
  private RobotSubsystems robot;

  public ShootOverStage(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.shooter.prepareShootOverStage();
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.isPivotAtSetpoint()
        && this.robot.shooter.getAnyNoteSensor();
  }
}
