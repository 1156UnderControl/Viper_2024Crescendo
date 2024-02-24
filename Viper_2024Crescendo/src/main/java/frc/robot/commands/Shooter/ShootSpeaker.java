package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.subsystems.Limelight;

public class ShootSpeaker extends Command {
  private RobotSubsystems robot;
  ControlBoard controlBoard = ControlBoard.getInstance();

  public ShootSpeaker(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.shooter.prepareSpeakerShot();
    if (this.robot.isRobotAbleToShoot()) {
      Limelight.ledOn();
    } else {
      Limelight.ledBlink();
    }
  }

  @Override
  public boolean isFinished() {
    return this.robot.isRobotAbleToShoot() || this.controlBoard.getShoot().getAsBoolean();
  }
}
