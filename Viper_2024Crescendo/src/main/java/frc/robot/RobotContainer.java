// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Autonomous.AutonomousAim;
import frc.robot.commands.Autonomous.AutonomousAimSteal;
import frc.robot.commands.Autonomous.AutonomousAimSteal1;
import frc.robot.commands.Autonomous.AutonomousAimSteal2;
import frc.robot.commands.Autonomous.AutonomousAimStealAuto2;
import frc.robot.commands.Autonomous.AutonomousAimSubwoofer;
import frc.robot.commands.Autonomous.AutonomousCollect;
import frc.robot.commands.Autonomous.AutonomousCollectAligning;
import frc.robot.commands.Autonomous.AutonomousDeployOnlyIntake;
import frc.robot.commands.Autonomous.AutonomousGoToNote;
import frc.robot.commands.Autonomous.AutonomousPassThrough;
import frc.robot.commands.Autonomous.AutonomousSecured;
import frc.robot.commands.Autonomous.AutonomousShoot;
import frc.robot.commands.Autonomous.Auto.AutoTestComand;
import frc.robot.commands.States.ClimbPosition;
import frc.robot.commands.States.CollectHPposition;
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

  public ControlBoard controlBoard = ControlBoard.getInstance();

  public RobotContainer() {
    this.swerve = SwerveSubsystem.getInstance(new File(Filesystem.getDeployDirectory(),
        "swerve/swerve"));
    this.robot = new RobotSubsystems();
    NamedCommands.registerCommand("Intake", new AutonomousCollect(robot));
    NamedCommands.registerCommand("Intake Aligning", new AutonomousCollectAligning(robot));
    NamedCommands.registerCommand("Pass Through", new AutonomousPassThrough(robot));
    NamedCommands.registerCommand("Shoot", new AutonomousShoot(robot).andThen(Commands.waitSeconds(0.1)));
    NamedCommands.registerCommand("Aim Shoot Subwoofer", new AutonomousAimSubwoofer(robot));
    NamedCommands.registerCommand("Aim Shoot Subwoofer Side", new AutonomousAimSubwoofer(robot));
    NamedCommands.registerCommand("Aim Shoot 1", new AutonomousAim(robot, 0));
    NamedCommands.registerCommand("Aim Shoot 2", new AutonomousAim(robot, 1));
    NamedCommands.registerCommand("Aim Shoot 3", new AutonomousAim(robot, 2));
    NamedCommands.registerCommand("Aim Shoot 4", new AutonomousAim(robot, 3));
    NamedCommands.registerCommand("Aim Shoot Steal auto 2 notes", new AutonomousAimStealAuto2(robot));
    NamedCommands.registerCommand("Aim Shoot Steal", new AutonomousAimSteal(robot));
    NamedCommands.registerCommand("Aim Shoot Steal 1", new AutonomousAimSteal1(robot));
    NamedCommands.registerCommand("Aim Shoot Steal 2", new AutonomousAimSteal2(robot));
    NamedCommands.registerCommand("Deploy Intake", new AutonomousDeployOnlyIntake(robot));
    NamedCommands.registerCommand("Secured", new AutonomousSecured(robot));
    NamedCommands.registerCommand("Go To Note", new AutonomousGoToNote(robot));

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
    this.controlBoard.getIntakeFromHP().onTrue(new CollectHPposition(robot));
    this.controlBoard.getSecured().onTrue(new SecuredPosition(robot));
    this.controlBoard.getPrepareAmp().and(() -> robot.shooter.hasNote())
        .onTrue(new ShootAmpPosition(robot, swerve));
    this.controlBoard.getPrepareSpeaker().and(() -> robot.shooter.hasNote())
        .onTrue(new ShootSpeakerPosition(robot));
    this.controlBoard.getDoNotShoot().and(() -> !robot.preparingShoot).and(() -> robot.shooter.hasNote())
        .onTrue(parallel(
            new ShootOverStagePosition(robot), new SwerveAlignShootOverStage(swerve)));
    this.controlBoard.release().onTrue(new ClimbPosition(robot));
    this.controlBoard.expell().onTrue(new ExpellPosition(robot));
    this.controlBoard.falseNegative().onTrue(new InstantCommand(() -> robot.shooter.setHasNoteTrue()));
    this.controlBoard.falsePositive().onTrue(new InstantCommand(() -> robot.shooter.setHasNoteFalse()));
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("New Path");
    return new AutoTestComand(robot, autoChooser);
    // return AutoBuilder.followPath(path1);
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
