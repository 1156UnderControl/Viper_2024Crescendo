package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimInterpolation extends Command {
  private edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
  private RobotSubsystems robot;
  private int shootNumber;

  public AutonomousAimInterpolation(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim Shoot:" + this.robot.shooter.isPivotAtSetpoint();
    this.robot.intake.goToIntakePosition();
    this.robot.intake.stopIntaking();
    this.robot.shooter.prepareSpeakerShot();
    timer.start();
  }

  @Override
  public void execute() {
    this.robot.shooter.prepareSpeakerShot();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    // return robot.shooter.isPivotAtSetpoint() && (robot.shooter.getNoteSensor() ||
    // timer.get() > 1);
    return false;
  }
}
