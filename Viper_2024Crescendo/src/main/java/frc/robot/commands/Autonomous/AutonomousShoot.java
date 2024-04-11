package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousShoot extends Command {
  private RobotSubsystems robot;
  private edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

  public AutonomousShoot(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Shoot";
    this.robot.shooter.shoot();
    this.robot.intake.intake();
    this.robot.intake.goToIntakePosition();
    timer.start();
  }

  @Override
  public void execute() {
    this.robot.shooter.shoot();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.hasShooted() || timer.get() > 2;
  }
}
