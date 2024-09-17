package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class AutonomousLongPassThrough extends Command {
  private RobotSubsystems robot;

  public AutonomousLongPassThrough(RobotSubsystems robot) {
    this.robot = robot;
    addRequirements(robot);
  }

  @Override
  public void initialize() {
    this.robot.intake.goToIntakePosition();
    this.robot.intake.intake();
  }

  @Override
  public void execute() {
    if (robot.intake.isSecureForShooter()) {
      this.robot.shooter.longPassThrough();
    }
    this.robot.intake.intake();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
