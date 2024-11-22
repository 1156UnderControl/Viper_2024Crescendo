// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
<<<<<<< HEAD
import frc.robot.Joysticks.controllers.OneController;
import frc.robot.commands.Autonomous.Auto.Auto;
=======
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillLEFTDynamic;
import frc.robot.commands.Autonomous.Auto.AutoStealOverkillRIGHTDynamic;
>>>>>>> main
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAim;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimAtAngle;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimAtSpeakerMovingOFF;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimAtSpeakerMovingON;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimInterpolation;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSteal;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSteal1;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSteal2;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimStealAuto2;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSubwoofer;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousAimSwerveSpeaker;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousCollect;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousDeployOnlyIntake;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousGoToNote;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousPassThrough;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousSecured;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousShoot;
import frc.robot.commands.Autonomous.Auto.Commands.AutonomousShootInterpolation;
import frc.robot.commands.States.CollectPosition;
import frc.robot.commands.States.ExpellPosition;
import frc.robot.commands.States.SecuredPosition;
import frc.robot.commands.States.ShootAmpPosition;
import frc.robot.commands.States.ShootOverStagePosition;
import frc.robot.commands.States.ShootSpeakerPosition;
import frc.robot.commands.Swerve.SwerveAlignShootOverStage;
import frc.robot.commands.Swerve.SwerveAlignSpeaker;
import frc.robot.commands.Swerve.SwerveTeleopControl;
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
  public final RobotSubsystems robot;

  private final SendableChooser<Command> autoChooser;

  public OneController controlBoard = OneController.getInstance();

  public RobotContainer() {
    this.swerve = SwerveSubsystem.getInstance(new File(Filesystem.getDeployDirectory(),
        "swerve/swerve"));
    this.robot = new RobotSubsystems();
    NamedCommands.registerCommand("Intake", new AutonomousCollect(robot));
    NamedCommands.registerCommand("Aim At Speaker ON", new AutonomousAimAtSpeakerMovingON(robot));
    NamedCommands.registerCommand("Aim At Speaker OFF", new AutonomousAimAtSpeakerMovingOFF(robot));
    NamedCommands.registerCommand("Pass Through", new AutonomousPassThrough(robot));
    NamedCommands.registerCommand("Shoot", new AutonomousShoot(robot).andThen(Commands.waitSeconds(0.1)));
    NamedCommands.registerCommand("Shoot Interpolation",
        new AutonomousShootInterpolation(robot).andThen(Commands.waitSeconds(0.1)));
    NamedCommands.registerCommand("Aim Swerve Speaker", new AutonomousAimSwerveSpeaker(robot));
    NamedCommands.registerCommand("Aim Shoot Subwoofer", new AutonomousAimSubwoofer(robot));
    NamedCommands.registerCommand("Aim Shoot Subwoofer Side", new AutonomousAimSubwoofer(robot));
    NamedCommands.registerCommand("Aim Shoot 1", new AutonomousAim(robot, 0));
    NamedCommands.registerCommand("Aim Shoot 2", new AutonomousAim(robot, 1));
    NamedCommands.registerCommand("Aim Shoot 3", new AutonomousAim(robot, 2));
    NamedCommands.registerCommand("Aim Shoot 4", new AutonomousAim(robot, 3));
    NamedCommands.registerCommand("Aim Shoot Interpolation", new AutonomousAimInterpolation(robot));
    NamedCommands.registerCommand("Aim Shoot Steal auto 2 notes", new AutonomousAimStealAuto2(robot));
    NamedCommands.registerCommand("Aim Shoot Steal", new AutonomousAimSteal(robot));
    NamedCommands.registerCommand("Aim Shoot Steal 1", new AutonomousAimSteal1(robot));
    NamedCommands.registerCommand("Aim Shoot Steal 2", new AutonomousAimSteal2(robot));
    NamedCommands.registerCommand("Deploy Intake", new AutonomousDeployOnlyIntake(robot));
    NamedCommands.registerCommand("Secured", new AutonomousSecured(robot));
    NamedCommands.registerCommand("Go To Note", new AutonomousGoToNote(robot));
    NamedCommands.registerCommand("Aim Last Note Overkill", new AutonomousAimAtAngle(robot, 90));

    autoChooser = AutoBuilder.buildAutoChooser("None");
    SmartDashboard.putData("Aut Chooser", autoChooser);
    configureBindings();
    swerve.setDefaultCommand(new SwerveTeleopControl(swerve));
  }

  private void configureBindings() {
    this.configureSwerveBindings();
    this.configureRobotBindings();
    this.controlBoard.getIntake().and(() -> RobotSubsystems.shootFinished).onTrue(new CollectPosition(robot, swerve));
  }

  private void configureSwerveBindings() {
    this.controlBoard.resetGyro().onTrue(new InstantCommand(swerve::zeroGyro));
    this.controlBoard.setHeadingFront().onTrue(new InstantCommand(swerve::setAngle0));
    this.controlBoard.setHeadingBack().onTrue(new InstantCommand(swerve::setAngle180));
    this.controlBoard.setHeadingLeft().onTrue(new InstantCommand(swerve::setAngle90));
    this.controlBoard.setHeadingRight().onTrue(new InstantCommand(swerve::setAngleNeg90));
    this.controlBoard.getSecured().onTrue(new SwerveTeleopControl(swerve));
    this.controlBoard.getPrepareSpeaker().and(() -> robot.shooter.hasNote())
        .onTrue(new SwerveAlignSpeaker(swerve));
  }

  private void configureRobotBindings() {
    this.controlBoard.getSecured().onTrue(new SecuredPosition(robot));
    this.controlBoard.getPrepareAmp().and(() -> robot.shooter.hasNote())
        .onTrue(new ShootAmpPosition(robot, swerve));
    this.controlBoard.getPrepareSpeaker().and(() -> robot.shooter.hasNote())
        .onTrue(new ShootSpeakerPosition(robot));
    this.controlBoard.getDoNotShoot().and(() -> !robot.preparingShoot).and(() -> robot.shooter.hasNote())
        .onTrue(parallel(
            new ShootOverStagePosition(robot), new SwerveAlignShootOverStage(swerve)));
    this.controlBoard.expell().onTrue(new ExpellPosition(robot));
    this.controlBoard.falseNegative().onTrue(new InstantCommand(() -> robot.shooter.setHasNoteTrue()));
    this.controlBoard.falsePositive().onTrue(new InstantCommand(() -> robot.shooter.setHasNoteFalse()));
  }

  public Command getAutonomousCommand() {
    switch (autoChooser.getSelected().getName()) {
      case "Steal Overkill Left (Red) Dynamic":
        return new AutoStealOverkillLEFTDynamic(this.robot, this.autoChooser);
      case "Steal Overkill RIGHT (Red) Dynamic":
        return new AutoStealOverkillRIGHTDynamic(this.robot, this.autoChooser);
      default:
        return this.autoChooser.getSelected();
    }
  }

  public void setRobotToBrake() {
    this.robot.setBrakeToRobot();
  }

  public void setRobotToCoast() {
    this.robot.setCoastToRobot();
  }

  public void setHeadingCorrection(boolean active) {
    swerve.setHeadingCorrection(active);
  }

}
