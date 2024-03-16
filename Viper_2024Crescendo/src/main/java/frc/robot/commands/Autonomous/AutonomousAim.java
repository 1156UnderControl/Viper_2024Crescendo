package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAim extends Command {
  private RobotSubsystems robot;
  private int shootNumber;
  private double[] shotPositions = { 72, 60, 65, 60 };

  public AutonomousAim(RobotSubsystems robot, int shootNumber) {
    this.robot = robot;
    this.addRequirements(robot);
    this.shootNumber = shootNumber;
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim Shoot:" + this.shootNumber;
    double shootPosition = shotPositions[this.shootNumber];
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
    return robot.shooter.isPivotAtSetpoint() && robot.shooter.getNoteSensor();
  }
}
