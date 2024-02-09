// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandConstant;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;

  public boolean trackSpeaker;

  SwerveController controller;

  Rotation2d angleAlign = new Rotation2d(Units.degreesToRadians(90));

  MedianFilter filter = new MedianFilter(5);

  PIDController pidTrackApril = new PIDController(CommandConstant.P_TrackSpeaker, CommandConstant.I_TrackSpeaker,
      CommandConstant.D_TrackSpeaker);

  Pose2d poseVision;
  Field2d aprField2d = new Field2d();

  public SwerveSubsystem(File directory) {

    SmartDashboard.putData("POSE APRIL", aprField2d);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

    try {

      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_VEL,
          SwerveConstants.SteeringConversionFactor,
          SwerveConstants.DriveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      swerveDrive.setGyro(new Rotation3d(0, 0, Units.degreesToRadians(180)));
    } else {
      swerveDrive.zeroGyro();
    }

    // swerveDrive.addVisionMeasurement(poseVision, Timer.getFPGATimestamp());
    setupPathPlanner();
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    if (!trackSpeaker) {
      LimelightHelpers.setPipelineIndex("", 0);
      // poseVision = LimelightHelpers.getBotPose2d_wpiBlue("");
      // aprField2d.setRobotPose(poseVision);
    } else {
      LimelightHelpers.setPipelineIndex("", 1);
    }

  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            AutoConstants.TranslationPID,
            // Translation PID constants
            AutoConstants.angleAutoPID,
            // Rotation PID constants
            4.5,
            // Max module speed, in m/s
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()
        // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.chassisVelocityCorrection = true;
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    SmartDashboard.putNumber("Pose inicial X", initialHolonomicPose.getX());
    SmartDashboard.putNumber("Pose inicial Y", initialHolonomicPose.getY());
    SmartDashboard.putNumber("Angulo Inicial", initialHolonomicPose.getRotation().getDegrees());
    swerveDrive.resetOdometry(initialHolonomicPose);

  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void setHeadingCorrection(boolean active) {
    swerveDrive.setHeadingCorrection(active);
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    SmartDashboard.putNumber("Angle Joy", Math.atan2(headingX, headingY));
    SmartDashboard.putNumber("Heading Rad", getHeading().getRadians());

    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
        getHeading().getRadians(), SwerveConstants.MAX_VEL);
  }

  // Resolve o Skew
  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        new Rotation2d(speeds.omegaRadiansPerSecond * dt * 7.5));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public void addVisionReading() {

    // Pose2d poseVision = new Pose2d(3, 3, Rotation2d.fromDegrees(0));
    /*
     * poseVision = new Pose2d(
     * new Translation2d(
     * filter.calculate(poseVision.getX()),
     * filter.calculate(poseVision.getY())),
     * new Rotation2d(
     * filter.calculate(poseVision.getRotation().getDegrees())));
     */
    if (LimelightHelpers.getTV("")) {

      swerveDrive.addVisionMeasurement(poseVision, Timer.getFPGATimestamp());
    }
  }

  public double trackSpeaker() {
    LimelightHelpers.setPipelineIndex("", 1);
    return pidTrackApril.calculate(LimelightHelpers.getTX(""), 0);
  }

  public void setTrackSpeaker() {
    trackSpeaker = true;
  }

  public void setStopTrackSpeaker() {
    trackSpeaker = false;
  }

  /*
   * ANGLE CONTROL
   */

  public void setAngle90() {
    angleAlign = new Rotation2d(Units.degreesToRadians(90));
  }

  public void setAngle180() {
    angleAlign = new Rotation2d(Units.degreesToRadians(180));
  }

  public void setAngleNeg90() {
    angleAlign = new Rotation2d(Units.degreesToRadians(-90));
  }

  public void setAngle0() {
    angleAlign = new Rotation2d(Units.degreesToRadians(0));
  }

  public void setAngle45() {
    angleAlign = new Rotation2d(Units.degreesToRadians(45));
  }

  public Rotation2d getAngleSP() {
    return angleAlign;
  }
}
