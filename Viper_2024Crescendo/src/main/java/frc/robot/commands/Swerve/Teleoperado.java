
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LimelightHelpers;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * Classe que calcula a partir da entrada do gamepad a saída do swerve
 */
public class Teleoperado extends Command {
  // Variáveis que guardam nossas funções do gamepad
  DoubleSupplier y;
  DoubleSupplier x;
  DoubleSupplier boostActivate;
  DoubleSupplier triggerSpeed;
  DoubleSupplier turnX;
  DoubleSupplier turnY;

  // Objetos necessárias para acessar funções e variáveis
  SwerveSubsystem swerve;
  SwerveController controller;

  // Variáveis que guardam a translação e velocidade angular do swerve
  Translation2d translation;
  ChassisSpeeds speedsWithAng;
  double angle;
  double omega;

  boolean initRotation;

  PIDController pidTrackApril;

  public Teleoperado(SwerveSubsystem swerve, DoubleSupplier y, DoubleSupplier x, DoubleSupplier turnX,
      DoubleSupplier turnY, DoubleSupplier triggerSpeed, DoubleSupplier boostActivate) {
    // Aqui atribuimos as funções e subsistema
    this.y = y;
    this.x = x;
    this.boostActivate = boostActivate;
    this.triggerSpeed = triggerSpeed;

    this.turnX = turnX;
    this.turnY = turnY;
    this.swerve = swerve;
    controller = swerve.getSwerveController(); // Obtemos o controlador do swerve
    // Adiciona a tração como requerimento
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initRotation = true;
  }

  // Abaixo calculamos os valores de saída a partir dos nossos inputs
  @Override
  public void execute() {

    double xVelocity = x.getAsDouble();
    double yVelocity = y.getAsDouble();
    double angXjoystick = turnX.getAsDouble();
    double angYjoystick = turnY.getAsDouble();
    double angX;
    double angY;

    double triggerSpeed = Math.pow(this.triggerSpeed.getAsDouble(), 2);
    boolean notUsingJoystick = frc.Java_Is_UnderControl.Util.Util.inRange(angXjoystick, -JoystickConstants.DEADBAND,
        0.2)
        && frc.Java_Is_UnderControl.Util.Util.inRange(angYjoystick, -JoystickConstants.DEADBAND,
            JoystickConstants.DEADBAND);

    if (notUsingJoystick) {
      angX = swerve.getAngleSP().getCos();
      angY = swerve.getAngleSP().getSin();
    } else {
      angX = angXjoystick;
      angY = angYjoystick;
    }

    if (triggerSpeed < 0.6) {
      xVelocity = triggerSpeed * xVelocity;
      yVelocity = triggerSpeed * yVelocity;
    } else {
      if (boostActivate.getAsDouble() < 0.2) {
        xVelocity = xVelocity * 0.6;
        yVelocity = yVelocity * 0.6;
      }
    }

    SmartDashboard.putNumber("xVel", xVelocity);
    SmartDashboard.putNumber("yVel", yVelocity);
    SmartDashboard.putNumber("Turn X", angX);
    SmartDashboard.putNumber("Turn Y", angY);
    SmartDashboard.putNumber("angPre", swerve.getAngleSP().getDegrees());

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      speedsWithAng = swerve.getTargetSpeeds(-xVelocity, -yVelocity, -angX, -angY);
    } else {
      speedsWithAng = swerve.getTargetSpeeds(xVelocity, yVelocity, angX, angY);
    }

    speedsWithAng = swerve.discretize(speedsWithAng);

    if (initRotation && !notUsingJoystick) {
      if (angX == 0 && angY == 0) {
        double firstLoopHeading = swerve.getHeading().getRadians();

        speedsWithAng = swerve.getTargetSpeeds(0, 0, Math.cos(firstLoopHeading), Math.sin(firstLoopHeading));
      }

      initRotation = false;
    }

    translation = SwerveController.getTranslation2d(speedsWithAng);

    if (SwerveConstants.ACELL_CORREC) {
      translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
          SwerveConstants.LOOP_TIME, SwerveConstants.ROBOT_MASS,
          List.of(SwerveConstants.CHASSIS),
          swerve.getSwerveDriveConfiguration());
    }

    if (swerve.trackSpeaker && LimelightHelpers.getTV("")) {

      speedsWithAng = new ChassisSpeeds(translation.getX(), translation.getY(), swerve.trackSpeaker());
    }

    SmartDashboard.putNumber("Angular Vel", speedsWithAng.omegaRadiansPerSecond);

    swerve.drive(translation, speedsWithAng.omegaRadiansPerSecond, true);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
