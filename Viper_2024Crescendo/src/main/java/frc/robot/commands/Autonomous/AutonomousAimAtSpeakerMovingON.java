package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSubsystems;

public class AutonomousAimAtSpeakerMovingON extends Command {
  private RobotSubsystems robot;

  public AutonomousAimAtSpeakerMovingON(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    Robot.autonomousState = "Aim At Speaker Moving ON";
    this.robot.swerve.overrideRotationAuto(true);
  }

  @Override
  public void execute() {
    this.robot.swerve.overrideRotationAuto(true);
  }

  @Override
  public void end(boolean interrupted) {
    this.robot.swerve.overrideRotationAuto(true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
