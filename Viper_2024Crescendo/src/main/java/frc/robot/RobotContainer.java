// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.swervedrive.drivebase.Teleoperado;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/swerve"));

  private final SendableChooser<Command> autoChooser;

  XboxController driverXbox = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("Shoot", new PrintCommand("SHOOT"));
    NamedCommands.registerCommand("Intake", new PrintCommand("INTAKING"));

    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Aut Chooser", autoChooser);

    configureBindings();

    drivebase.setDefaultCommand(new Teleoperado(drivebase,
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), JoystickConstants.DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), JoystickConstants.DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY(),
        () -> driverXbox.getRightTriggerAxis(),
        () -> driverXbox.getLeftTriggerAxis()));

  }

  private void configureBindings() {
    new JoystickButton(driverXbox, 1).onTrue(new InstantCommand(drivebase::setAngleNeg90));
    new JoystickButton(driverXbox, 2).onTrue(new InstantCommand(drivebase::setAngle180));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::setAngle0));
    new JoystickButton(driverXbox, 4).onTrue(new InstantCommand(drivebase::setAngle90));
    new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(drivebase::setAngle45));
    new JoystickButton(driverXbox, 6).toggleOnTrue((Commands.run(() -> drivebase.addVisionReading())));
    new JoystickButton(driverXbox, 6).toggleOnFalse(Commands.none());
    new JoystickButton(driverXbox, 7).onTrue(new InstantCommand(drivebase::setTrackSpeaker));
    new JoystickButton(driverXbox, 8).onTrue(new InstantCommand(drivebase::setStopTrackSpeaker));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void setHeadingCorrection(boolean active) {
    drivebase.setHeadingCorrection(active);
  }

}
