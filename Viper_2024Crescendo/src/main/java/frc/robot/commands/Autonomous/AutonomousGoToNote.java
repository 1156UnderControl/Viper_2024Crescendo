package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;

public class AutonomousGoToNote extends Command {
  private edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
  private RobotSubsystems robot;

  public AutonomousGoToNote(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    this.robot.intake.goToIntakePosition();
    this.robot.intake.intake();
    if (this.robot.intake.isSecureForShooter()) {
      this.robot.shooter.goToIntakePosition();
      this.robot.shooter.intakeAuto();
    }
  }

  @Override
  public void execute() {
    this.robot.intake.intake();
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
    return robot.shooter.hasNote();
  }
}
