package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.subsystems.LEDs.LedColor;
import frc.robot.subsystems.LEDs.LedSubsystem;

public class GoToIntakeIntake extends Command {
  private RobotSubsystems robot;
  private LedSubsystem led;

  public GoToIntakeIntake(RobotSubsystems robot, LedSubsystem led) {
    this.robot = robot;
    this.led = led;
    this.addRequirements(robot, led);
  }

  @Override
  public void initialize() {
    this.robot.intake.intake();
    this.led.setSolidColor(LedColor.RED);
  }

  @Override
  public void execute() {
    this.robot.intake.goToIntakePosition();
  }

  @Override
  public boolean isFinished() {
    return this.robot.intake.isSecureForShooter();
  }
}
