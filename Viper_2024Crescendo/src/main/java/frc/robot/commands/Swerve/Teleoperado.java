
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

public class Teleoperado extends Command {
  DoubleSupplier y;
  DoubleSupplier x;
  boolean boostActivate;
  DoubleSupplier triggerSpeed;
  DoubleSupplier turnX;
  DoubleSupplier turnY;

  SwerveSubsystem swerve;
  SwerveController controller;
  ControlBoard mControlBoard = ControlBoard.getInstance();

  Translation2d translation;
  ChassisSpeeds speedsWithAng;
  double angle;
  double omega;

  boolean initRotation;

  PIDController pidTrackApril;

  public Teleoperado(SwerveSubsystem swerve, DoubleSupplier y, DoubleSupplier x, DoubleSupplier turnX,
      DoubleSupplier turnY, DoubleSupplier triggerSpeed, DoubleSupplier boostActivate) {
    this.y = y;
    this.x = x;
    this.boostActivate = mControlBoard.turboActivate();
    this.triggerSpeed = triggerSpeed;

    this.turnX = turnX;
    this.turnY = turnY;
    this.swerve = swerve;
    controller = swerve.getSwerveController();
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    initRotation = true;
  }

  @Override
  public void execute() {

    double xVelocity = x.getAsDouble();
    double yVelocity = y.getAsDouble();
    double angXjoystick = turnX.getAsDouble();
    double angYjoystick = turnY.getAsDouble();
    double angX;
    double angY;

    double triggerSpeed = Math.pow(this.triggerSpeed.getAsDouble(), 2);

    if (mControlBoard.notUsingJoystick()) {
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
      if (!boostActivate) {
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

    if (initRotation && !mControlBoard.notUsingJoystick()) {
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
