// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Autonomous.AutonomousAim;
import frc.robot.commands.Autonomous.AutonomousAimSubwoofer;
import frc.robot.commands.Autonomous.AutonomousCollect;
import frc.robot.commands.Autonomous.AutonomousShoot;
import frc.robot.commands.Autonomous.AutonomousVisionUpdate;
import frc.robot.commands.States.CollectPosition;
import frc.robot.commands.States.SecuredPosition;
import frc.robot.commands.States.ShootAmpPosition;
import frc.robot.commands.States.ShootSpeakerPosition;
import frc.robot.commands.Swerve.SwerveAlignSpeaker;
import frc.robot.commands.Swerve.SwerveTeleopControl;
import frc.robot.subsystems.LEDs.LedSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerve;

  public final RobotSubsystems robot = new RobotSubsystems();

  public final LedSubsystem led = new LedSubsystem();

  private final SendableChooser<Command> autoChooser;

  public ControlBoard mControlBoard = ControlBoard.getInstance();

  public RobotContainer() {
    this.swerve = SwerveSubsystem.getInstance(new File(Filesystem.getDeployDirectory(),
        "swerve/swerve"));
    NamedCommands.registerCommand("Vision Update", new AutonomousVisionUpdate(swerve));
    NamedCommands.registerCommand("Intake", new AutonomousCollect(robot));
    NamedCommands.registerCommand("Shoot", new AutonomousShoot(robot).andThen(Commands.waitSeconds(0.1)));
    NamedCommands.registerCommand("Aim Shoot Subwoofer", new AutonomousAimSubwoofer(robot));
    NamedCommands.registerCommand("Aim Shoot 1", new AutonomousAim(robot, 0));
    NamedCommands.registerCommand("Aim Shoot 2", new AutonomousAim(robot, 1));
    NamedCommands.registerCommand("Aim Shoot 3", new AutonomousAim(robot, 2));
    NamedCommands.registerCommand("Aim Shoot 4", new AutonomousAim(robot, 3));
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Aut Chooser", autoChooser);
    configureBindings();
    swerve.setDefaultCommand(new SwerveTeleopControl(swerve));
  }

  private void configureBindings() {
    mControlBoard.setHeadingFront().onTrue(new InstantCommand(swerve::setAngle0));
    mControlBoard.setHeadingBack().onTrue(new InstantCommand(swerve::setAngle180));
    mControlBoard.setHeadingLeft().onTrue(new InstantCommand(swerve::setAngle90));
    mControlBoard.setHeadingRight().onTrue(new InstantCommand(swerve::setAngleNeg90));
    mControlBoard.getIntake().onTrue(new CollectPosition(robot, swerve));
    mControlBoard.getSecured().onTrue(new SecuredPosition(robot));
    mControlBoard.getSecured().onTrue(new SwerveTeleopControl(swerve));
    mControlBoard.getPrepareAmp().and(() -> robot.shooter.hasNote()).onTrue(new ShootAmpPosition(robot));
    mControlBoard.getPrepareSpeaker().and(() -> robot.shooter.hasNote()).onTrue(new ShootSpeakerPosition(robot));
    mControlBoard.getPrepareSpeaker().and(() -> robot.shooter.hasNote()).onTrue(new SwerveAlignSpeaker(swerve));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void setHeadingCorrection(boolean active) {
    swerve.setHeadingCorrection(active);
  }

}
