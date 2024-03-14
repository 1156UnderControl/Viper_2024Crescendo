package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousShoot extends Command {
  private RobotSubsystems robot;

  public AutonomousShoot(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Shoot";
    this.robot.intake.intake();
    this.robot.intake.goToIntakePosition();
  }

  @Override
  public void execute() {
    this.robot.shooter.shoot();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.hasShooted();
  }
}
