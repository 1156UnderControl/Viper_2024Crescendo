package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class SecuredPosition extends SequentialCommandGroup {

  public SecuredPosition(ShooterSubsystem shooter, IntakeSubsystem intake) {
    addCommands(

    );
  }
}
