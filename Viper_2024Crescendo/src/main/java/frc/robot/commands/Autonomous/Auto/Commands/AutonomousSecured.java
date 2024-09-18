package frc.robot.commands.Autonomous.Auto.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousSecured extends Command {
  private RobotSubsystems robot;

  public AutonomousSecured(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "SECURED";
    this.robot.shooter.goToSecuredPosition();
  }

  @Override
  public void execute() {
    if (this.robot.shooter.isSecureForIntake()) {
      this.robot.intake.goToSecuredPosition();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return robot.shooter.isPivotAtSetpoint() && robot.intake.isAtSetpoint();
  }
}
