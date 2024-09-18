package frc.robot.commands.Autonomous.Auto.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimSubwoofer extends Command {
  private RobotSubsystems robot;
  private double shotPosition = 49;
  private edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

  public AutonomousAimSubwoofer(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim Shoot Subwoofer";
    this.robot.intake.goToIntakePosition();
    this.robot.intake.stopIntaking();
    timer.start();
  }

  @Override
  public void execute() {
    this.robot.shooter.setManualPivotPosition(shotPosition, 7000);
  }

  @Override
  public void end(boolean interrupted) {
    this.robot.shooter.stopFeeder();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return robot.shooter.isPivotAtSetpoint() && (robot.shooter.getNoteSensor() || timer.get() > 0.8);
  }
}
