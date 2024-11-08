package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.controllers.OneController;

public class ShootOverStage extends Command {
  private RobotSubsystems robot;
  private OneController controller = OneController.getInstance();

  public ShootOverStage(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {
    this.robot.shooter.prepareShootOverStage();
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.isPivotAtSetpoint() && controller.getShoot().getAsBoolean()
        && this.robot.shooter.getAnyNoteSensor();
  }
}
