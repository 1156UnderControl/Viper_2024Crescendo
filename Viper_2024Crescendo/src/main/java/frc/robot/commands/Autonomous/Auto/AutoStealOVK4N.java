package frc.robot.commands.Autonomous.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.either;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class AutoStealOVK4N extends SequentialCommandGroup {
  private RobotSubsystems robot = new RobotSubsystems();

  public AutoStealOVK4N(RobotSubsystems robot, SendableChooser<Command> autoChooser) {
    this.robot = robot;
    addCommands(
        either(new Got1And2Note(this.robot), AutoBuilder.followPath(null),
            () -> ShooterSubsystem.getNote[0] == true && ShooterSubsystem.getNote[1] == true),
        either(new Got1Note(this.robot), AutoBuilder.followPath(null),
            () -> ShooterSubsystem.getNote[0] == true && ShooterSubsystem.getNote[1] == false),
        either(new Got2Note(this.robot), AutoBuilder.followPath(null),
            () -> ShooterSubsystem.getNote[0] == false && ShooterSubsystem.getNote[1] == true));
  }
}
