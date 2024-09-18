package frc.robot.commands.Autonomous.Auto.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimAtSpeakerMovingOFF extends Command {
  private RobotSubsystems robot;

  public AutonomousAimAtSpeakerMovingOFF(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim At Speaker Moving OFF";
    this.robot.swerve.overrideRotationAuto(false);
  }

  @Override
  public void execute() {
    this.robot.swerve.overrideRotationAuto(false);
  }

  @Override
  public void end(boolean interrupted) {
    this.robot.swerve.overrideRotationAuto(false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
