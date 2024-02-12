package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class GoToSecuredIntake extends Command {
  private RobotSubsystems robot;

  public GoToSecuredIntake(RobotSubsystems robot) {
    this.robot = robot;
    addRequirements(this.robot);
  }

  @Override
  public void execute() {
    this.robot.intake.securedPosition();
  }

  @Override
  public boolean isFinished() {
    return this.robot.intake.atSetpoint();
  }
}
