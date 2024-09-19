package frc.robot.commands.Autonomous.Auto.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class AutonomousOVKPassThrough extends Command {
  private RobotSubsystems robot = new RobotSubsystems();
  public static int noteNumberOVK = -1;

  public AutonomousOVKPassThrough(RobotSubsystems robot) {
    this.robot = robot;
    addRequirements(this.robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Collect";
    this.robot.intake.goToIntakePosition();
    this.robot.intake.intake();
    noteNumberOVK += 1;
  }

  @Override
  public void execute() {
    if (this.robot.intake.isSecureForShooter()) {
      this.robot.shooter.passThrough();
      if (this.robot.shooter.hasPassedNoteThrough()) {
        ShooterSubsystem.getNote[noteNumberOVK] = true;
      }
    }
    this.robot.intake.intake();
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return ShooterSubsystem.getNote[0];
  }
}
