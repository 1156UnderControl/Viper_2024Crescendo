package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.controllers.OneController;

public class ShootSpeaker extends Command {
  private RobotSubsystems robot;
  private OneController controller = OneController.getInstance();

  public ShootSpeaker(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.shooter.prepareSpeakerShot();
  }

  @Override
  public boolean isFinished() {
    return this.robot.isRobotAbleToShoot()
        || this.controller.getShoot().getAsBoolean() && robot.swerve.isSwerveInVelocityToShoot();
  }
}
