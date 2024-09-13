package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimAtLastNote extends Command {
  private RobotSubsystems robot;

  public AutonomousAimAtLastNote(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim at Last Note OVK";
    this.robot.swerve.aimSwerve(defineAngleToAlignAtNote());
    this.robot.intake.goToIntakePosition();
    this.robot.intake.intake();
    if (this.robot.intake.isSecureForShooter()) {
      this.robot.shooter.goToIntakePosition();
      this.robot.shooter.intakeAuto();
    }
  }

  @Override
  public void execute() {
    this.robot.swerve.aimSwerve(defineAngleToAlignAtNote());
    if (this.robot.intake.isSecureForShooter()) {
      this.robot.shooter.goToIntakePosition();
      this.robot.shooter.intakeAuto();
      this.robot.swerve.driveToNote();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return robot.swerve.isAtTargetHeading();
  }

  private double defineAngleToAlignAtNote() {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      return 100;
    } else {
      return -100;
    }
  }
}
