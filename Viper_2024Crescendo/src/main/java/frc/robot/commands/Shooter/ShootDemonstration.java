package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShootDemonstration extends Command {
  double setPointValue;
  double setPointSpeaker;
  double dpadValue = 0;
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
      while (controller.setAngleUP().getAsBoolean()) {
        setPointValue = dpadValue + shooterSubsystem.calculateGoalPivotSetpointSpeaker(setPointSpeaker);
        dpadValue++;
      }
      while (controller.setAngleDOWN().getAsBoolean()) {
        setPointValue = dpadValue + shooterSubsystem.calculateGoalPivotSetpointSpeaker(setPointSpeaker);
        dpadValue--;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return this.robot.shooter.isPivotAtSetpoint();
  }
}
