package frc.robot.commands.Autonomous.Auto.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimAtAngle extends Command {
  private RobotSubsystems robot;
  private double angleToAim;

  public AutonomousAimAtAngle(RobotSubsystems robot, double angle) {
    this.robot = robot;
    this.angleToAim = angle;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim at Angle:";
    this.robot.swerve.aimSwerve(defineAngleToAlign(angleToAim));
    this.robot.intake.goToIntakePosition();
    this.robot.intake.intake();
    if (this.robot.intake.isSecureForShooter()) {
      this.robot.shooter.goToIntakePosition();
      this.robot.shooter.intakeAuto();
    }
  }

  @Override
  public void execute() {
    this.robot.swerve.aimSwerve(defineAngleToAlign(angleToAim));
    if (this.robot.intake.isSecureForShooter()) {
      this.robot.shooter.goToIntakePosition();
      this.robot.shooter.intakeAuto();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return robot.swerve.isAtTargetHeading();
  }

  private double defineAngleToAlign(double angle) {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      return angle;
    } else {
      return -angle;
    }
  }
}
