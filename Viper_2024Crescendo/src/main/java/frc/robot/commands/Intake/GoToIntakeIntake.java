package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class GoToIntakeIntake extends Command {
  private RobotSubsystems robot;

  public GoToIntakeIntake(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    this.robot.intake.intake();
  }

  @Override
  public void execute() {
    this.robot.intake.goToIntakePosition();
  }

  @Override
  public boolean isFinished() {
    return this.robot.intake.isSecureForShooter();
  }
}
