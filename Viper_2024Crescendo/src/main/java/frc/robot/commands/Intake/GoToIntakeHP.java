package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotSubsystems;
import frc.robot.subsystems.LEDs.LedColor;

public class GoToIntakeHP extends Command {
  private RobotSubsystems robot;

  public GoToIntakeHP(RobotSubsystems robot) {
    this.robot = robot;
    this.addRequirements(robot);
  }

  @Override
  public void initialize() {
    this.robot.intake.intakeFromHPposition();
    this.robot.led.setSolidColor(LedColor.YELLOW);
  }

  @Override
  public void execute() {
    this.robot.intake.intakeFromHPposition();
  }

  @Override
  public boolean isFinished() {
    return this.robot.intake.isSecureForShooter();
  }
}
