// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.io.File;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Limelight.Limelight;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.Java_Is_UnderControl.Logging.Pose2dLogEntry;
import frc.Java_Is_UnderControl.PhotonVision.PhotonVision;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandsConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Joysticks.ControlBoard;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase implements ISwerve {

  private SwerveDrive swerveDrive;

  private ControlBoard controller = ControlBoard.getInstance();

  public boolean notUsingLLforOdometry = false;

  private final int MAX_LOST_SPEAKER_FRAMES = 2;

  private final int MAX_LOST_NOTE_FRAMES = 2;

  private int lostSpeakerFramesCounter = this.MAX_LOST_SPEAKER_FRAMES;

  private int lostNotesFrameCounter = this.MAX_LOST_NOTE_FRAMES;

  private double lastTxMeasurement;

  private static SwerveSubsystem swerveInstance = null;

  private double angleToAimAtSpeaker = 0;

  private double headingDegrees;

  private double headingRadians;

  private StabilizeChecker alignedToSpeakerStabilizeChecker = new StabilizeChecker();

  private Rotation2d angleAlign = new Rotation2d(Units.degreesToRadians(0));

  PIDController pidTrackNote = new PIDController(CommandsConstants.P_TrackNote, CommandsConstants.I_TrackNote,
      CommandsConstants.D_TrackNote);

  Pose2d poseVision2D = new Pose2d();

  Pose3d poseVision3D = new Pose3d();

  LimelightHelpers.PoseEstimate limelightMeasurement;

  Field2d aprField2d = new Field2d();

  private Pose2dLogEntry poseLogEntry = new Pose2dLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Pose");

  private Pose2dLogEntry poseVisionLogEntry = new Pose2dLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/PoseVision");

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  private DoubleLogEntry targetXSpeedLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/TargetSpeeds/X");

  private DoubleLogEntry targetYSpeedLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/TargetSpeeds/Y");

  private DoubleLogEntry targetOmegaSpeedLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/TargetSpeeds/Omega");

  private BooleanLogEntry isAlignedWithSpeakerLogEntry = new BooleanLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/isAligned");

  private BooleanLogEntry tvMeasurementLlLogEntry = new BooleanLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/tvLL");

  private DoubleLogEntry txMeasurementLlLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/txLL");

  private String state = "DRIVE_ALIGN_ANGLE_BUTTON";

  private StringLogEntry stateLogEntry = new StringLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/State");

  private PhotonVision photonVision;

  private Limelight limelight;

  private double targetNotePositionX;

  public static SwerveSubsystem getInstance(File directory) {
    if (swerveInstance == null) {
      swerveInstance = new SwerveSubsystem(directory);
    }
    return swerveInstance;
  }

  public static SwerveSubsystem getInstance() {
    if (swerveInstance == null) {
      DriverStation.reportError("YAGSL Configuration not provided", true);
    }
    return swerveInstance;
  }

  private SwerveSubsystem(File directory) {
    this.limelight = Limelight.getInstance();
    this.photonVision = new PhotonVision();
    SmartDashboard.putData("POSE APRIL", aprField2d);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    setupYAGSL_JSON(directory);
    setupPathPlanner();
    this.updateLogs();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("yaw tag", limelight.getYawTagSpeaker());
    swerveDrive.updateOdometry();
    Optional<EstimatedRobotPose> estimation = limelight
        .getEstimatedGlobalPose(swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition());
    headingDegrees = getHeading().getDegrees();
    headingRadians = getHeading().getRadians();
    SmartDashboard.putNumber("Heading Odometry", swerveDrive.getOdometryHeading().getDegrees());
    SmartDashboard.putNumber("Distance LL", limelight.getDistance());
    SmartDashboard.putNumber("Distance Odom", distanceFromSpeaker());
    if (estimation.isPresent()) {
      limelight.setPipeline(0);
      SmartDashboard.putNumber("tags", estimation.get().targetsUsed.size());
      poseVision2D = estimation.get().estimatedPose.toPose2d();
      poseVision3D = estimation.get().estimatedPose;
      if (estimation.get().targetsUsed.size() >= 2) {
        aprField2d.setRobotPose(poseVision2D);
        swerveDrive.addVisionMeasurement(poseVision2D, estimation.get().timestampSeconds);
      }
    }

    this.updateLogs();
  }

  @Override
  public void driveAlignAngleButton() {
    if (controller.notUsingJoystick()) {
      ChassisSpeeds speeds = getTargetSpeedsAngle(controller.getYtranslation(), controller.getXtranslation(),
          angleAlign.getDegrees());
      SmartDashboard.putString("Chassis Speed", speeds.toString());
      driveFieldOriented(speeds);
      this.state = "DRIVE_ALIGN_ANGLE_BUTTON";
    } else {
      driveAlignAngleJoy();
    }
  }

  @Override
  public void driveAlignAngleJoy() {
    ChassisSpeeds speeds = getTargetSpeedsAngleJoystick(controller.getYtranslation(), controller.getXtranslation(),
        controller.getCOS_Joystick(), controller.getSIN_Joystick());
    SmartDashboard.putString("Chassis Speed", speeds.toString());
    driveFieldOriented(speeds);
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
  }

  @Override
  public void driveAlignSpeaker() {
    if (this.controller.isForcingDriverControl()) {
      driveAlignAngleButton();
      return;
    }
    Translation2d angleSpeaker;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      angleSpeaker = getPose().getTranslation().minus(FieldConstants.redGoalPos);
    } else {
      angleSpeaker = getPose().getTranslation().minus(FieldConstants.blueGoalPos);
    }

    this.angleToAimAtSpeaker = angleSpeaker.rotateBy(new Rotation2d(Math.PI)).getAngle().getDegrees();

    ChassisSpeeds speeds = getTargetSpeedsAngle(controller.getYtranslation(), controller.getXtranslation(),
        this.angleToAimAtSpeaker);

    driveFieldOriented(speeds);
    this.state = "DRIVE_ALIGN_SPEAKER";
  }

  @Override
  public void driveAlignNote() {
    if (this.controller.isForcingDriverControl()) {
      driveAlignAngleButton();
      return;
    }
    boolean isDetectingNote = photonVision.hasTarget();
    if (isDetectingNote) {
      this.lostNotesFrameCounter = 0;
      this.targetNotePositionX = photonVision.targetPositionX();
      ChassisSpeeds speeds = getTargetSpeedsAngle(controller.getYtranslation(), controller.getXtranslation(),
          0);
      driveOmegaPidFieldOriented(speeds, 0, this.targetNotePositionX, this.pidTrackNote, false);
    } else {
      this.lostNotesFrameCounter++;
      if (this.lostNotesFrameCounter <= this.MAX_LOST_SPEAKER_FRAMES) {
        ChassisSpeeds speeds = getTargetSpeedsAngle(controller.getYtranslation(), controller.getXtranslation(),
            0);
        driveOmegaPidFieldOriented(speeds, 0, this.lastTxMeasurement, this.pidTrackNote, false);
      } else {
        driveAlignAngleButton();
      }
    }
    this.state = "DRIVE_ALIGN_NOTE";
  }

  public boolean isSwerveAlignedWithSpeaker() {
    boolean aligned = this.alignedToSpeakerStabilizeChecker.isStableInRange(
        swerveDrive.getOdometryHeading().getDegrees(),
        angleToAimAtSpeaker,
        SwerveConstants.DEADBAND_ALIGNED_WITH_SPEAKER);
    return aligned;
  }

  private void setupYAGSL_JSON(File directory) {
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_VEL,
          SwerveConstants.SteeringConversionFactor,
          SwerveConstants.DriveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
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

  private void driveFieldOriented(ChassisSpeeds speeds) {
    swerveDrive.driveFieldOriented(speeds);
    this.targetSpeeds = speeds;
  }

  private void driveOmegaPidFieldOriented(ChassisSpeeds speeds, double setpoint, double measurement, PIDController pid,
      boolean invertOmega) {
    ChassisSpeeds speedsPID = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        invertOmega ? -pid.calculate(measurement, setpoint) : pid.calculate(measurement, setpoint));
    SmartDashboard.putNumber("Omega PID", speedsPID.omegaRadiansPerSecond);
    swerveDrive.driveFieldOriented(speedsPID);
    this.targetSpeeds = speedsPID;
  }

  private void driveOmegaPidRobotOriented(ChassisSpeeds speeds, double setpoint, double measurement, PIDController pid,
      boolean invertOmega) {
    ChassisSpeeds speedsPID;
    speedsPID = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        invertOmega ? -pid.calculate(measurement, setpoint) : pid.calculate(measurement, setpoint));
    SmartDashboard.putNumber("Omega PID", speedsPID.omegaRadiansPerSecond);
    swerveDrive.drive(speedsPID);
    this.targetSpeeds = speedsPID;
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
    this.targetSpeeds = chassisSpeeds;
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

  private Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  private ChassisSpeeds getTargetSpeedsAngle(double xInput, double yInput, double angle) {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      xInput = -xInput;
      yInput = -yInput;
    }
    double angleTarget = angle;
    ChassisSpeeds targetSpeeds;
    SmartDashboard.putNumber("Angle Target Speeds", angleTarget);
    double angleRadians = Units.degreesToRadians(angle);
    targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angleRadians,
        headingRadians, SwerveConstants.MAX_VEL);
    return targetSpeeds;
  }

  private ChassisSpeeds getTargetSpeedsAngleJoystick(double xInput, double yInput, double headingX, double headingY) {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      xInput = -xInput;
      yInput = -yInput;
    }
    double angleTarget = Math.atan2(headingX, headingY);
    ChassisSpeeds targetSpeeds;
    SmartDashboard.putNumber("Angle Target Speeds", Units.radiansToDegrees(angleTarget));
    targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
        headingRadians, SwerveConstants.MAX_VEL);
    return targetSpeeds;
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

  public double distanceFromSpeaker() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return getPose().getTranslation().getDistance(FieldConstants.redGoalPos);
    } else {
      return getPose().getTranslation().getDistance(FieldConstants.blueGoalPos);
    }
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  /*
   * public double trackNote() {
   * return pidTrackNotes.calculate();
   * }
   */

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

  public void setAngleNeg45() {
    angleAlign = new Rotation2d(Units.degreesToRadians(-45));
  }

  private void updateLogs() {
    this.poseLogEntry.appendRadians(this.swerveDrive.getPose());
    this.poseVisionLogEntry.appendRadians(this.poseVision2D);
    this.targetXSpeedLogEntry.append(this.targetSpeeds.vxMetersPerSecond);
    this.targetYSpeedLogEntry.append(this.targetSpeeds.vyMetersPerSecond);
    this.targetOmegaSpeedLogEntry.append(this.targetSpeeds.omegaRadiansPerSecond);
    this.isAlignedWithSpeakerLogEntry.append(isSwerveAlignedWithSpeaker());
    this.txMeasurementLlLogEntry.append(this.lastTxMeasurement);
    this.tvMeasurementLlLogEntry.append(LimelightHelpers.getTV(""));
    this.stateLogEntry.append(this.state);
  }
}
