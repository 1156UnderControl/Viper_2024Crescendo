package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.subsystems.LEDs.LedColor;

public class StartIntakingIntake extends Command {
  private RobotSubsystems robot;

  public StartIntakingIntake(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    if (this.robot.shooter.isPivotAtSetpoint()) {
      this.robot.intake.intake();
    }
    this.robot.led.setSolidColor(LedColor.YELLOW);
  }

  @Override
  public void execute() {
    if (this.robot.shooter.isPivotAtSetpoint()) {
      this.robot.intake.intake();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
