package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.Limelight.Limelight;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToSecuredIntake;
import frc.robot.commands.Shooter.GoToSecuredShooter;
import frc.robot.subsystems.LEDs.LedColor;

public class SecuredPosition extends SequentialCommandGroup {

  public SecuredPosition(RobotSubsystems robot) {
    addCommands(
        new InstantCommand(() -> Limelight.ledOff()),
        new InstantCommand(() -> RobotSubsystems.shootFinished = false),
        new GoToSecuredShooter(robot),
        new GoToSecuredIntake(robot),
        Commands.either(new InstantCommand(() -> robot.led.setSolidColor(LedColor.WHITE)),
            new InstantCommand(() -> robot.led.setSolidColor(LedColor.OFF)),
            () -> robot.robotHasNote()));
  }
}
