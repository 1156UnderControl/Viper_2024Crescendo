package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousCollectAligning extends Command {
  private RobotSubsystems robot;

  public AutonomousCollectAligning(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Collect";
    this.robot.shooter.intakeAuto();
    this.robot.intake.goToIntakePosition();
    this.robot.intake.intake();
    this.robot.swerve.overrideRotationAuto(true);
  }

  @Override
  public void execute() {
    this.robot.swerve.overrideRotationAuto(true);
    this.robot.shooter.intakeAuto();
    this.robot.intake.intake();
  }

  @Override
  public void end(boolean interrupted) {
    this.robot.swerve.overrideRotationAuto(false);
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.hasNote();
  }
}
