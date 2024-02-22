package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimSubwoofer extends Command {
  private RobotSubsystems robot;
  private double shotPosition = 49;

  public AutonomousAimSubwoofer(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim Shoot Subwoofer";
    double shootPosition = this.shotPosition;
    this.robot.shooter.setManualPivotPosition(shootPosition);
    this.robot.intake.goToIntakePosition();
    this.robot.intake.stopIntaking();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return robot.shooter.isPivotAtSetpoint();
  }
}
