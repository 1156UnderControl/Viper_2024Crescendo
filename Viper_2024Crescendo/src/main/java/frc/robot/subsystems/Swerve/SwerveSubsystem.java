// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.io.File;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Limelight.Limelight;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.Java_Is_UnderControl.Logging.Pose2dLogEntry;
import frc.Java_Is_UnderControl.PhotonVision.PhotonVision;
import frc.Java_Is_UnderControl.Swerve.BaseSwerveConfig;
import frc.Java_Is_UnderControl.Swerve.BaseSwerveSubsystem;
import frc.Java_Is_UnderControl.Swerve.SwervePathPlannerConfig;
import frc.Java_Is_UnderControl.Util.StabilizeChecker;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandsConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Joysticks.ControlBoard;

public class SwerveSubsystem extends BaseSwerveSubsystem implements ISwerve {

  private ControlBoard controller = ControlBoard.getInstance();

  public boolean twoApriltagsDetected = false;

  public boolean alignedWithAngle;

  private final int MAX_LOST_SPEAKER_FRAMES = 2;

  private final int MAX_LOST_NOTE_FRAMES = 2;

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

  private Pose2dLogEntry poseVisionLogEntry = new Pose2dLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/PoseVision");

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  private BooleanLogEntry isAlignedWithSpeakerLogEntry = new BooleanLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/isAligned");

  private BooleanLogEntry isSwerveInAutoShootZoneLogEntry = new BooleanLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/isInAutoShootZone");

  private BooleanLogEntry isSwerveInAutoShootVelocityLogEntry = new BooleanLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/isInAutoShootVelocity");

  private BooleanLogEntry isSwerveInShootVelocityLogEntry = new BooleanLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/isInShootVelocity");

  private DoubleLogEntry angleToAimSpeakerLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/TargetSpeeds/Omega");

  private DoubleLogEntry distanceToNearestAmpLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/AutoAmp/DistanceToNearestAmp");

  private BooleanLogEntry isAlingedWithAmpLogEntry = new BooleanLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/AutoAmp/isAlignedWithAmp");

  private BooleanLogEntry tvMeasurementLlLogEntry = new BooleanLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/tvLL");

  private DoubleLogEntry txMeasurementLlLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/SpeakerAlign/txLL");

  private String state = "DRIVE_ALIGN_ANGLE_BUTTON";

  private StringLogEntry stateLogEntry = new StringLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/State");

  private PhotonVision photonVision;

  private Limelight limelight;

  private double targetNotePositionX;

  private PIDController xVelToPose = new PIDController(0.7, 0, 0);
  private PIDController yVelToPose = new PIDController(0.7, 0, 0);

  private PIDController xVelToAprilTag = new PIDController(0.01, 0, 0);
  private PIDController yVelToAprilTag = new PIDController(0.01, 0, 0);

  private PhotonVision shooterTopCamera = new PhotonVision("Microsoft_LifeCam_HD-3000");

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
    super(directory, new BaseSwerveConfig(
      SwerveConstants.MAX_VEL,
      SwerveConstants.MAX_ROTATION_VEL,
      Alliance.Red,
      new SwervePathPlannerConfig(
        AutoConstants.TranslationPID,
        AutoConstants.angleAutoPID,
        4.5)));
    this.limelight = Limelight.getInstance();
    this.photonVision = new PhotonVision();
    SmartDashboard.putData("POSE APRIL", aprField2d);
    this.updateLogs();
  }

  @Override
  public void periodic() {
    super.periodic();
    updateHeadingVariables();
    updateSmartDashboardValues();
    updateOdometryWithVision();
    this.updateLogs();
  }

  private void updateOdometryWithVision() {
    swerveDrive.updateOdometry();
    Optional<EstimatedRobotPose> estimation = limelight
        .getEstimatedGlobalPose(swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition());
    if (estimation.isPresent()) {
      EstimatedRobotPose fakeRobotPoseIfNull = new EstimatedRobotPose(new Pose3d(),
          0, new ArrayList<>(), PoseStrategy.LOWEST_AMBIGUITY);
      limelight.setPipeline(0);
      poseVision2D = estimation.orElse(fakeRobotPoseIfNull).estimatedPose.toPose2d();
      poseVision3D = estimation.orElse(fakeRobotPoseIfNull).estimatedPose;
      if (Robot.isInAutonomous) {
        OdometryCorrectionInAuto(estimation, fakeRobotPoseIfNull);
      } else {
        OdometryCorrectionInTeleOp(estimation, fakeRobotPoseIfNull);
      }
    }
  }

  private void OdometryCorrectionInTeleOp(Optional<EstimatedRobotPose> estimation,
      EstimatedRobotPose fakeRobotPoseIfNull) {
    if (estimation.get().targetsUsed.size() >= 2) {
      aprField2d.setRobotPose(poseVision2D);
      swerveDrive.addVisionMeasurement(poseVision2D, estimation.orElse(fakeRobotPoseIfNull).timestampSeconds);
      twoApriltagsDetected = true;
    } else if (poseVision2D.getTranslation().getDistance(getPose().getTranslation()) < 0.7
        && poseInTheField(poseVision2D)
        && limelight.getTV() && limelight.getDistance() < 2) {
      aprField2d.setRobotPose(poseVision2D);
      swerveDrive.addVisionMeasurement(poseVision2D, estimation.orElse(fakeRobotPoseIfNull).timestampSeconds);
      twoApriltagsDetected = false;
    } else {
      twoApriltagsDetected = false;
    }
  }

  private boolean poseInTheField(Pose2d pose) {
    boolean poseInXofField = pose.getX() > 0 && pose.getX() < 16.5;
    boolean poseInYofField = pose.getY() > 0 && pose.getY() < 8;
    return poseInXofField && poseInYofField;
  }

  private void OdometryCorrectionInAuto(Optional<EstimatedRobotPose> estimation,
      EstimatedRobotPose fakeRobotPoseIfNull) {
    if (estimation.get().targetsUsed.size() >= 2) {
      aprField2d.setRobotPose(poseVision2D);
      swerveDrive.addVisionMeasurement(poseVision2D, estimation.orElse(fakeRobotPoseIfNull).timestampSeconds);
      twoApriltagsDetected = true;
    }
  }

  private void updateHeadingVariables() {
    headingDegrees = getHeading().getDegrees();
    headingRadians = getHeading().getRadians();
  }

  private void updateSmartDashboardValues() {
    SmartDashboard.putString("state swerve", state);
    SmartDashboard.putString("Target Chassis Speeds", targetSpeeds.toString());
    SmartDashboard.putNumber("Heading Odometry", swerveDrive.getOdometryHeading().getDegrees());
    SmartDashboard.putNumber("Distance LL", limelight.getDistance());
    SmartDashboard.putNumber("Distance Odom", distanceFromSpeaker());
    SmartDashboard.putBoolean("Aligned With Speaker", isSwerveAlignedWithSpeaker());
    SmartDashboard.putBoolean("Swerve at Velocity to auto shoot", isSwerveInVelocityToAutoShoot());
    SmartDashboard.putBoolean("Swerve in zone to auto shoot", isSwerveInAutoShootZone());
  }

  @Override
  public void driveAlignAngleButton() {
    if (controller.notUsingJoystick()) {
      ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(), controller.getXtranslation());
      this.driveFieldOrientedLockedAngle(desiredSpeeds, angleAlign);
      this.state = "DRIVE_ALIGN_ANGLE_BUTTON";
    } else {
      this.driveAlignAngleJoy();
    }
  }

  @Override
  public void driveAlignAngleJoy() {
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(), controller.getXtranslation());
    double angleTarget = Math.atan2(controller.getCOS_Joystick(), controller.getSIN_Joystick());
    this.driveFieldOrientedLockedAngle(desiredSpeeds, Rotation2d.fromRadians(angleTarget));
    this.state = "DRIVE_ALIGN_ANGLE_JOY";
  }

  @Override
  public void driveAlignSpeaker() {
    if (this.controller.isForcingDriverControl()) {
      this.driveAlignAngleButton();
      return;
    }
    Translation2d angleSpeaker;
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      angleSpeaker = this.getPose().getTranslation().minus(FieldConstants.redGoalPos);
      this.angleToAimAtSpeaker = angleSpeaker.rotateBy(new Rotation2d(Math.PI)).getAngle().getDegrees();
    } else {
      angleSpeaker = this.getPose().getTranslation().minus(FieldConstants.blueGoalPos);
      this.angleToAimAtSpeaker = angleSpeaker.getAngle().getDegrees();
    }
    ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(), controller.getXtranslation());
    this.driveFieldOrientedLockedAngle(desiredSpeeds, Rotation2d.fromDegrees(this.angleToAimAtSpeaker));
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
      ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(), controller.getXtranslation());
      driveOmegaPidFieldOriented(desiredSpeeds, 0, this.targetNotePositionX, this.pidTrackNote, false);
    } else {
      this.lostNotesFrameCounter++;
      if (this.lostNotesFrameCounter <= this.MAX_LOST_SPEAKER_FRAMES) {
        ChassisSpeeds desiredSpeeds = this.inputsToChassisSpeeds(controller.getYtranslation(), controller.getXtranslation());
        driveOmegaPidFieldOriented(desiredSpeeds, 0, this.lastTxMeasurement, this.pidTrackNote, false);
      } else {
        driveAlignAngleButton();
      }
    }
    this.state = "DRIVE_ALIGN_NOTE";
  }

  @Override
  public void driveToAmp() {
    Pose2d targetPose;
    if (this.getPose().getX() > FieldConstants.midFieldLine) {
      targetPose = FieldConstants.redAmp;
    } else {
      targetPose = FieldConstants.blueAmp;
    }
    driveToPose(
        new Pose2d(targetPose.getX(), targetPose.getY(),
            new Rotation2d(Units.degreesToRadians(angleAlign.getDegrees()))));
    this.state = "DRIVE_TO_AMP";
  }

  private void driveToPose(Pose2d targetPose) {
    if (this.controller.isForcingDriverControl()) {
      driveAlignAngleButton();
      return;
    }
    angleAlign = targetPose.getRotation();
    Pose2d currentPose = this.getPose();
    double[] speeds = pidXyVelocity(targetPose.getX(), currentPose.getX(),
        targetPose.getY(), currentPose.getY());
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(speeds[0], speeds[1], 0);
    this.driveFieldOrientedLockedAngle(desiredSpeeds, targetPose.getRotation());  
  }

  @Override
  public void alignWithAmpAprilTag() {
    double angleAlignAmp = defineAngleToAlignWithAmp();
    this.alignWithAprilTag(0.0, SwerveConstants.ALIGNED_WITH_AMP_GOAL, angleAlignAmp);
    this.state = "ALIGN_WITH_AMP_APRILTAG";
  }

  private double defineAngleToAlignWithAmp() {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      return 90;
    } else {
      return -90;
    }
  }

  private void alignWithAprilTag(double targetX, double targetY, double targetAngle) {
    if (this.controller.isForcingDriverControl()) {
      this.driveAlignAngleButton();
      return;
    }
    if (!this.shooterTopCamera.hasTarget()) {
      this.driveAlignAngleButton();
      return;
    }
    double aprilTagX = this.shooterTopCamera.targetPositionX();
    double aprilTagY = this.shooterTopCamera.targetPositionY();
    double velocityX = -xVelToAprilTag.calculate(aprilTagX, targetX);
    double velocityY = yVelToAprilTag.calculate(aprilTagY, targetY);
    ChassisSpeeds targetSpeeds = this.inputsToChassisSpeeds(velocityX, velocityY);
    this.driveRobotOrientedLockedAngle(targetSpeeds, Rotation2d.fromDegrees(targetAngle));
  }

  @Override
  public double getDistanceToNearestAmp() {
    if (this.getPose().getX() > FieldConstants.midFieldLine) {
      return this.getPose().getTranslation().getDistance(FieldConstants.redAmp.getTranslation());
    } else {
      return this.getPose().getTranslation().getDistance(FieldConstants.blueAmp.getTranslation());
    }
  }

  @Override
  public boolean isSwerveAlignedWithSpeaker() {
    boolean aligned = this.alignedToSpeakerStabilizeChecker.isStableInRange(
        headingDegrees,
        angleToAimAtSpeaker,
        SwerveConstants.DEADBAND_ALIGNED_WITH_SPEAKER);
    return aligned;
  }

  @Override
  public boolean isAlignedWithAngle() {
    alignedWithAngle = Util.inRange(Math.abs(headingDegrees), Math.abs(angleAlign.getDegrees()) - 5,
        Math.abs(angleAlign.getDegrees()) + 5);
    return alignedWithAngle;
  }

  @Override
  public void setAngleToAmp() {
    double angleAlignAmp = defineAngleToAlignWithAmp();
    this.angleAlign = new Rotation2d(Units.degreesToRadians(angleAlignAmp));
  }

  @Override
  public boolean isAtAmpPosition() {
    Translation2d targetAmpPosition;
    if (this.getPose().getX() > FieldConstants.midFieldLine) {
      targetAmpPosition = FieldConstants.redAmp.getTranslation();
    } else {
      targetAmpPosition = FieldConstants.blueAmp.getTranslation();
    }
    Translation2d currentPosition = this.getPose().getTranslation();
    boolean isAlignedInX = Math
        .abs(currentPosition.getX() - targetAmpPosition.getX()) < SwerveConstants.DEADBAND_AT_AMP_POSITION_X;
    boolean isAlignedInY = Math
        .abs(currentPosition.getY() - targetAmpPosition.getY()) < SwerveConstants.DEADBAND_AT_AMP_POSITION_Y;

    SmartDashboard.putBoolean("isAlignedInX", isAlignedInX);
    SmartDashboard.putBoolean("isAlignedInY", isAlignedInY);
    return isAlignedInX && isAlignedInY;
  }

  @Override
  public boolean isAlignedWithAmp() {
    return this.isAlignedWithAprilTag(
        0,
        SwerveConstants.ALIGNED_WITH_AMP_GOAL,
        SwerveConstants.DEADBAND_ALIGNED_WITH_AMP_X,
        SwerveConstants.DEADBAND_ALIGNED_WITH_AMP_Y);
  }

  private boolean isAlignedWithAprilTag(double targetX, double targetY, double deadbandX, double deadbandY) {
    if (!this.shooterTopCamera.hasTarget()) {
      return false;
    }
    double aprilTagX = this.shooterTopCamera.targetPositionX();
    double aprilTagY = this.shooterTopCamera.targetPositionY();
    boolean isAlignedInX = Util.inRange(aprilTagX, targetX - deadbandX, targetX + deadbandX);
    boolean isAlignedInY = aprilTagY > -6;
    boolean alignedWithAngle = Util.inRange(Math.abs(headingDegrees), Math.abs(angleAlign.getDegrees()) - 5,
        Math.abs(angleAlign.getDegrees()) + 5);
    return isAlignedInX && isAlignedInY && alignedWithAngle;
  }

  @Override
  public boolean isSwerveInAutoShootZone() {
    boolean isInFrontOfSpeaker = this.getPose().getY() > 2.9;
    boolean isNearSpeaker = distanceFromSpeaker() < 3.5;
    return isNearSpeaker && isInFrontOfSpeaker;
  }

  @Override
  public boolean isSwerveInVelocityToAutoShoot() {
    double velocity = this.getAbsoluteRobotVelocity();
    return velocity < 1;
  }

  @Override
  public boolean isSwerveInVelocityToShoot() {
    double velocity = this.getAbsoluteRobotVelocity();
    return velocity < 2.5;
  }

  // private void driveFieldOriented(ChassisSpeeds speeds) {
  //   ChassisSpeeds speedsRotate;
  //   if (this.controller.rotateLeft().getAsBoolean()) {
  //     speedsRotate = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 5);
  //   } else if (this.controller.rotateRight().getAsBoolean()) {
  //     speedsRotate = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -5);
  //   } else {
  //     speedsRotate = speeds;
  //   }
  //   swerveDrive.driveFieldOriented(speedsRotate);
  //   this.targetSpeeds = speeds;
  // }

  // private void driveRobotOriented(ChassisSpeeds speeds) {
  //   ChassisSpeeds speedsRotate;
  //   if (this.controller.rotateLeft().getAsBoolean()) {
  //     speedsRotate = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 8);
  //   } else if (this.controller.rotateRight().getAsBoolean()) {
  //     speedsRotate = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -8);
  //   } else {
  //     speedsRotate = speeds;
  //   }
  //   swerveDrive.drive(speedsRotate);
  //   this.targetSpeeds = speeds;
  // }

  private void driveOmegaPidFieldOriented(ChassisSpeeds speeds, double setpoint, double measurement, PIDController pid,
      boolean invertOmega) {
    ChassisSpeeds speedsPID = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        invertOmega ? -pid.calculate(measurement, setpoint) : pid.calculate(measurement, setpoint));
    SmartDashboard.putNumber("Omega PID", speedsPID.omegaRadiansPerSecond);
    swerveDrive.driveFieldOriented(speedsPID);
    this.targetSpeeds = speedsPID;
  }

  public double distanceFromSpeaker() {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
    if (alliance == Alliance.Red) {
      return getPose().getTranslation().getDistance(FieldConstants.redGoalPos);
    } else {
      return getPose().getTranslation().getDistance(FieldConstants.blueGoalPos);
    }
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
    angleAlign = new Rotation2d(Units.degreesToRadians(35));
  }

  public void setAngleNeg45() {
    angleAlign = new Rotation2d(Units.degreesToRadians(-45));
  }

  protected void updateLogs() {
    this.poseVisionLogEntry.appendRadians(this.poseVision2D);
    this.isAlignedWithSpeakerLogEntry.append(isSwerveAlignedWithSpeaker());
    this.isSwerveInAutoShootZoneLogEntry.append(isSwerveInAutoShootZone());
    this.isSwerveInShootVelocityLogEntry.append(isSwerveInVelocityToShoot());
    this.isSwerveInAutoShootVelocityLogEntry.append(isSwerveInVelocityToAutoShoot());
    this.distanceToNearestAmpLogEntry.append(getDistanceToNearestAmp());
    this.isAlingedWithAmpLogEntry.append(isAtAmpPosition());
    this.angleToAimSpeakerLogEntry.append(this.angleToAimAtSpeaker);
    this.txMeasurementLlLogEntry.append(this.lastTxMeasurement);
    this.tvMeasurementLlLogEntry.append(limelight.getTV());
    this.stateLogEntry.append(this.state);
  }

  private double[] pidXyVelocity(double goalPositionX, double currentPositionX, double goalPositionY,
      double currentPositionY) {
    double[] XYvel = {
        xVelToPose.calculate(currentPositionX, goalPositionX),
        yVelToPose.calculate(currentPositionY, goalPositionY) };

    SmartDashboard.putNumber("Goal X Pose AMP", goalPositionX);
    SmartDashboard.putNumber("Goal Y Pose AMP", goalPositionY);
    SmartDashboard.putNumber("Current X Pose", currentPositionX);
    SmartDashboard.putNumber("Current Y Pose", currentPositionY);
    return XYvel;
  }

}
