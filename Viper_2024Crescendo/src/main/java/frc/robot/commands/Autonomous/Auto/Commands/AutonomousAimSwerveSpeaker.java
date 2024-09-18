package frc.robot.commands.Autonomous.Auto.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimSwerveSpeaker extends Command {
  private RobotSubsystems robot;

  public AutonomousAimSwerveSpeaker(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim Swerve Speaker";
    this.robot.swerve.alignSpeaker();
    this.robot.shooter.prepareSpeakerShot();
  }

  @Override
  public void execute() {
    this.robot.swerve.alignSpeaker();
    this.robot.shooter.prepareSpeakerShot();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return robot.swerve.isSwerveAlignedWithSpeaker();
  }
}
