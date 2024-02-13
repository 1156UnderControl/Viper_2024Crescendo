package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class GoToAmpIntake extends Command {
  private RobotSubsystems robot;

  public GoToAmpIntake(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.intake.freewayShooter();
  }

  @Override
  public boolean isFinished() {
    return this.robot.intake.isSecureForShooter();
  }
}
