package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Intake.GoToSecuredIntake;
import frc.robot.commands.Shooter.GoToSecuredShooter;
import frc.robot.commands.Shooter.ReleaseClimber;

public class ClimbPosition extends SequentialCommandGroup {
  ControlBoard mControlBoard = ControlBoard.getInstance();

  public ClimbPosition(RobotSubsystems robot) {
    addCommands(
        new GoToSecuredShooter(robot),
        new GoToSecuredIntake(robot),
        new ReleaseClimber(robot),
        Commands.waitUntil(mControlBoard.climb()),
        new ClimbPosition(robot));
  }
}
