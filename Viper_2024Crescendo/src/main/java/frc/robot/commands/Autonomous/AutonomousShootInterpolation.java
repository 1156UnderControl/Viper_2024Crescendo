package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousShootInterpolation extends Command {
  private RobotSubsystems robot;
  private boolean isShooting = false;

  public AutonomousShootInterpolation(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Shoot";
    this.robot.shooter.prepareSpeakerShot();
    this.robot.intake.intake();
    this.robot.intake.goToIntakePosition();
    this.isShooting = false;
  }

  @Override
  public void execute() {
    if (!isShooting) {
      this.robot.shooter.prepareSpeakerShot();
    }
    if (robot.shooter.isPivotAtSetpoint() && robot.shooter.isFlywheelAtSetpoint()) {
      isShooting = true;
      this.robot.shooter.shoot();
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.robot.shooter.shoot();
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.hasShooted();
  }
}
