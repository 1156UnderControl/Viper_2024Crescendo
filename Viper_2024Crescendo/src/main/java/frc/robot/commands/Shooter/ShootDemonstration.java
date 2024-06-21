package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShootDemonstration extends Command {
  double setPointValue;
  double setPointInterpolation;
  double dpadValue = 0;
  long delayMiliseconds = 100;
  private RobotSubsystems robot;
  private ControlBoard controller = ControlBoard.getInstance();
  private ShooterSubsystem shooterSubsystem;

  public ShootDemonstration(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void execute() {

    this.robot.shooter.prepareDemonstrationShoot(setPointValue);

    if (controller.getPrepareSpeaker().getAsBoolean()) {
      if (controller.setAngleUP().getAsBoolean()) {
        while (setPointValue > ShooterConstants.setpoints.LIMIT_MIN_DEMONSTRATION) {
          setPointValue = dpadValue + shooterSubsystem.calculateGoalPivotSetpointSpeaker(setPointInterpolation);
          dpadValue--;
        }
      }
      if (controller.setAngleDOWN().getAsBoolean()) {
        while (setPointValue < ShooterConstants.setpoints.LIMIT_MAX_DEMONSTRATION) {
          setPointValue = dpadValue + shooterSubsystem.calculateGoalPivotSetpointSpeaker(setPointInterpolation);
          dpadValue++;
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return this.robot.isRobotAbleToShoot()
        || this.controller.getShoot().getAsBoolean() && robot.swerve.isSwerveInVelocityToShoot();
  }
}
