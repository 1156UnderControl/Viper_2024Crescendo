package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.controllers.OneController;
import frc.robot.commands.Intake.GoToClimbIntake;
import frc.robot.commands.Shooter.GoToSecuredShooter;
import frc.robot.commands.Shooter.PullClimber;
import frc.robot.commands.Shooter.ReleaseClimber;

public class ClimbPosition extends SequentialCommandGroup {
  OneController controller = OneController.getInstance();

  public ClimbPosition(RobotSubsystems robot) {
    addCommands(
        new GoToClimbIntake(robot),
        new GoToSecuredShooter(robot),
        new ReleaseClimber(robot),
        new PullClimber(robot));
  }
}
