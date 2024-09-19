package frc.robot.commands.Autonomous.Auto.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousPassThrough extends Command {
  private RobotSubsystems robot;

  public AutonomousPassThrough(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "PASS_THROUGH";
    this.robot.intake.goToIntakePosition();
    this.robot.intake.intake();
  }

  @Override
  public void execute() {
    if (this.robot.intake.isSecureForShooter()) {
      this.robot.shooter.passThrough();
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
