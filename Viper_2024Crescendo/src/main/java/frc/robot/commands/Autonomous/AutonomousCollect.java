package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousCollect extends Command {
  private RobotSubsystems robot;

  public AutonomousCollect(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Collect";
    this.robot.shooter.cruise();
    this.robot.shooter.intake();
    this.robot.intake.goToIntakePosition();
    this.robot.intake.intake();
  }

  @Override
  public void execute() {
    this.robot.intake.intake();
    this.robot.shooter.intakeAuto();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.hasNote();
  }
}
