package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;

public class PhotonVisionPoseEstimator implements PoseEstimator {

  PhotonPoseEstimator photonPoseEstimator;

  boolean only2TagsMeasurements = false;

  private AprilTagFieldLayout fieldAprilTagLayout;

  private CustomPose2dLogger detectedPoseLogger;

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition) {
    this.fieldAprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    this.fieldAprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(this.fieldAprilTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraPosition);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    this.detectedPoseLogger = new CustomPose2dLogger("/Vision/MultiCameraEstimator/DetectedPose");
  }

  public PhotonVisionPoseEstimator(PhotonCamera camera, Transform3d cameraPosition, boolean only2TagsMeasurements) {
    this.fieldAprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    this.fieldAprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    this.photonPoseEstimator = new PhotonPoseEstimator(this.fieldAprilTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraPosition);
    this.photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    this.photonPoseEstimator.setReferencePose(referencePose);
    Optional<EstimatedRobotPose> photonPoseEstimation = this.photonPoseEstimator.update();
    if (!photonPoseEstimation.isPresent()) {
      return Optional.empty();
    }
    PoseEstimation poseEstimation = convertPhotonPoseEstimation(photonPoseEstimation.get());
    if (only2TagsMeasurements && poseEstimation.numberOfTargetsUsed < 2) {
      return Optional.empty();
    }

    detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    return Optional.of(poseEstimation);
  }

  private PoseEstimation convertPhotonPoseEstimation(EstimatedRobotPose photonPoseEstimation) {
    return new PoseEstimation(photonPoseEstimation.estimatedPose, photonPoseEstimation.timestampSeconds,
        photonPoseEstimation.targetsUsed.size());
  }

}
