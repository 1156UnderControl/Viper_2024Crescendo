package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;

public class MultiCameraPoseEstimator implements PoseEstimator {

  List<PoseEstimator> poseEstimatorsList;

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  public MultiCameraPoseEstimator(List<PoseEstimator> poseEstimators) {
    this.poseEstimatorsList = poseEstimators;
    this.detectedPoseLogger = new CustomPose2dLogger("/Vision/MultiCameraEstimator/DetectedPose");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    ArrayList<Pose2d> pose2Ds = new ArrayList<>();
    ArrayList<PoseEstimation> poseEstimations = new ArrayList<>();
    for (PoseEstimator poseEstimator : this.poseEstimatorsList) {
      Optional<PoseEstimation> possiblePoseEstimation = poseEstimator.getEstimatedPose(referencePose);
      if (possiblePoseEstimation.isPresent()) {
        PoseEstimation poseEstimation = possiblePoseEstimation.get();
        for (int i = 0; i < poseEstimation.numberOfTargetsUsed; i++) {
          pose2Ds.add(poseEstimation.estimatedPose.toPose2d());
          poseEstimations.add(poseEstimation);
        }
      }
    }
    if (poseEstimations.isEmpty()) {
      return Optional.empty();
    }
    pose2Ds.sort(new ComparatorPose2D());
    poseEstimations.sort(new ComparatorPoseEstimation());
    Pose2d pose2D = frc.Java_Is_UnderControl.Util.Util.medianPose2Dlist(pose2Ds);
    PoseEstimation poseEstimation = frc.Java_Is_UnderControl.Util.Util.medianPoseEstimationsList(poseEstimations);
    detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    return Optional.of(poseEstimation);
  }

}
