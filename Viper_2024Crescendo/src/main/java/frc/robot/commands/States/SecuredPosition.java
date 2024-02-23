package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.Intake.GoToSecuredIntake;
import frc.robot.commands.Shooter.GoToSecuredShooter;

public class SecuredPosition extends SequentialCommandGroup {

  public SecuredPosition(RobotSubsystems robot) {
    addCommands(
        new InstantCommand(() -> RobotSubsystems.shootFinished = false),
        new GoToSecuredShooter(robot),
        new GoToSecuredIntake(robot));
  }
}
