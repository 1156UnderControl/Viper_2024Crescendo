package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Intake.GoToAmpIntake;
import frc.robot.commands.Shooter.PrepareForAmp;

public class ShootAmpPosition extends SequentialCommandGroup {
  ControlBoard mmBoard = ControlBoard.getInstance();

  public ShootAmpPosition(RobotSubsystems robot) {
    addCommands(
        new GoToAmpIntake(robot),
        new PrepareForAmp(robot),
        Commands.waitUntil(mmBoard.getShootAmp()),
        Commands.runOnce(() -> robot.shooter.shoot(), robot),
        Commands.waitSeconds(1));
  }
}
