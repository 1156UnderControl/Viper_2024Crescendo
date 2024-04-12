package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;
import frc.robot.Constants.ShooterConstants;

public class AutonomousAim extends Command {
  private RobotSubsystems robot;
  private int shootNumber;
  // private double[] shotPositions = { 72, 60, 65, 60 }; V1
  private double[] shotPositions = { 69, 60, 67, 70 };

  public AutonomousAim(RobotSubsystems robot, int shootNumber) {
    this.robot = robot;
    this.addRequirements(robot);
    this.shootNumber = shootNumber;
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim Shoot:" + this.shootNumber;
    double shootPosition = shotPositions[this.shootNumber];
    this.robot.shooter.setManualPivotPosition(shootPosition, ShooterConstants.setpoints.VELOCITY_SHOOT_SPEAKER);
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
