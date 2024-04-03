package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers.PoseEstimate;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;

public class LimelightPoseEstimator implements PoseEstimator {

  String limelightName;

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  public LimelightPoseEstimator(String limelightName) {
    this.limelightName = limelightName;
    this.isDetectingLogger = new CustomBooleanLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/IsDetectingTags");
    this.detectedPoseLogger = new CustomPose2dLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/DetectedPose");
    this.numberOfDetectedTagsLogger = new CustomDoubleLogger(
        "/Vision/LimelightPoseEstimator/" + limelightName + "/NumberOfDetectedTags");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    if (!LimelightHelpers.getTV(this.limelightName)) {
      this.isDetectingLogger.append(false);
      this.numberOfDetectedTagsLogger.append(0);
      return Optional.empty();
    }
    PoseEstimate limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.limelightName);
    PoseEstimation poseEstimation = convertPoseEstimate(limelightPoseEstimate);
    this.isDetectingLogger.append(true);
    this.detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    this.numberOfDetectedTagsLogger.append(poseEstimation.numberOfTargetsUsed);
    return Optional.of(poseEstimation);
  }

  private PoseEstimation convertPoseEstimate(PoseEstimate limelightPoseEstimate) {
    return new PoseEstimation(new Pose3d(limelightPoseEstimate.pose), limelightPoseEstimate.timestampSeconds,
        limelightPoseEstimate.tagCount);
  }

}
