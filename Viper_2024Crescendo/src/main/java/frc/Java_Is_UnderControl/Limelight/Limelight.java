// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Java_Is_UnderControl.Limelight;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Limelight {

  private static Limelight instance;

  private PhotonCamera camera;

  private AprilTagFieldLayout fieldAprils = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public EstimatedRobotPose estimatedRobotPose;

  private Transform3d robotToCam = new Transform3d(new Translation3d(-0.6, 0.0, 0.26),
      new Rotation3d(0, Units.degreesToRadians(24), Math.PI)); // Cam mounted facing forward, half a meter forward of
                                                               // center, half a meter up from center.

  PhotonPoseEstimator photonPoseEstimator;

  public static Limelight getInstance() {
    if (instance == null) {
      instance = new Limelight();
      return instance;
    } else {
      return instance;
    }
  }

  private Limelight() {
    this("limelight");
    fieldAprils.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    photonPoseEstimator = new PhotonPoseEstimator(fieldAprils,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
  }

  public Limelight(String cameraName) {
    this.camera = new PhotonCamera(cameraName);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d estimatedRobotPose) {
    photonPoseEstimator.setReferencePose(estimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public static boolean notUsingLLforOdometry = false;
  public static boolean tvAfterTweak = false;

  public double getTX() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {

    }
    return 0;
  }

  public boolean getTV() {
    var result = camera.getLatestResult();
    return result.hasTargets();
  }

  public void setPipeline(int index) {
    this.camera.setPipelineIndex(0);
  }

  public void ledBlink() {
    camera.setLED(VisionLEDMode.kBlink);
  }

  public void ledOn() {
    camera.setLED(VisionLEDMode.kOn);
  }

  public void ledOff() {
    camera.setLED(VisionLEDMode.kOff);
  }

  private static final double limelightMountAngleDegreesDEG = 24;

  private static final double limelightLensHeightMeters = 0.2599;

  private static final double goalHeightMeters = 1.368552;

  public double getDistance() {
    var result = camera.getLatestResult();
    double distance = 0;
    if (result.hasTargets()) {
      distance = PhotonUtils.calculateDistanceToTargetMeters(
          limelightLensHeightMeters,
          goalHeightMeters,
          Units.degreesToRadians(limelightMountAngleDegreesDEG),
          Units.degreesToRadians(result.getBestTarget().getPitch()));
    }
    return distance;
  }
}
