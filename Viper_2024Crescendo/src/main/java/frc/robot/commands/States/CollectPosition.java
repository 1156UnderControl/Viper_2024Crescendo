package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.Limelight.Limelight;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToIntakeIntake;
import frc.robot.commands.Shooter.CollectToShooter;
import frc.robot.commands.Swerve.SwerveAlignNote;
import frc.robot.subsystems.LEDs.LedColor;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {
  Limelight limelight = Limelight.getInstance();

  public CollectPosition(RobotSubsystems robot, SwerveSubsystem swerve) {
    addCommands(
        new InstantCommand(() -> robot.preparingShoot = false),
        new GoToIntakeIntake(robot),
        new ParallelRaceGroup(new CollectToShooter(robot), new SwerveAlignNote(swerve)),
        new InstantCommand(() -> robot.led.setBlink(LedColor.GREEN, 3, LedColor.WHITE)),
        new InstantCommand(() -> limelight.ledBlink()),
        Commands.runOnce(() -> robot.intake.stopIntaking()));
  }
}
