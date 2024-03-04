package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimSteal extends Command {
  private RobotSubsystems robot;
  private double shotPosition = 66;

  public AutonomousAimSteal(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim Shoot Steal";
    this.robot.intake.goToIntakePosition();
    this.robot.intake.stopIntaking();
    this.robot.shooter.setManualPivotPosition(shotPosition);
  }

  @Override
  public void execute() {
    this.robot.shooter.setManualPivotPosition(shotPosition);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return robot.shooter.isPivotAtSetpoint() && robot.shooter.getNoteSensor();
  }
}
