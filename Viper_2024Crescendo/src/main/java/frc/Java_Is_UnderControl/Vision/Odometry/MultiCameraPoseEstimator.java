package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomBooleanLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomPose2dLogger;

public class MultiCameraPoseEstimator implements PoseEstimator {

  List<PoseEstimator> poseEstimatorsList;

  PoseEstimation arrayPosesEstimations[];

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  public MultiCameraPoseEstimator(List<PoseEstimator> poseEstimators) {
    this.poseEstimatorsList = poseEstimators;
    this.detectedPoseLogger = new CustomPose2dLogger("/Vision/MultiCameraEstimator/DetectedPose");
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    ArrayList<Pose2d> poseEstimations = new ArrayList<>();
    for (PoseEstimator poseEstimator : this.poseEstimatorsList) {
      Optional<PoseEstimation> possiblePoseEstimation = poseEstimator.getEstimatedPose(referencePose);
      if (possiblePoseEstimation.isPresent()) {
        PoseEstimation poseEstimation = possiblePoseEstimation.get();
        for (int i = 0; i < poseEstimation.numberOfTargetsUsed; i++) {
          poseEstimations.add(poseEstimation.estimatedPose.toPose2d());
        }
      }
    }
    poseEstimations.sort(new ComparatorPose());
    PoseEstimation poseEstimation;
    if (arrayPosesEstimations.length % 2 == 0) {
      poseEstimation = averageTwoPoseEstimations(arrayPosesEstimations[arrayPosesEstimations.length / 2],
          arrayPosesEstimations[arrayPosesEstimations.length / 2 - 1]);
    } else {
      poseEstimation = arrayPosesEstimations[arrayPosesEstimations.length / 2];
    }
    detectedPoseLogger.appendRadians(poseEstimation.estimatedPose.toPose2d());
    return Optional.of(poseEstimation);
  }

  private PoseEstimation averageTwoPoseEstimations(PoseEstimation pose1, PoseEstimation pose2) {
    double x = (pose1.estimatedPose.getX() + pose2.estimatedPose.getX()) / 2;
    double y = (pose1.estimatedPose.getY() + pose2.estimatedPose.getY()) / 2;
    double rot = (pose1.estimatedPose.getRotation().getAngle() + pose2.estimatedPose.getRotation().getAngle()) / 2;
    int numberOfTargets = (pose1.numberOfTargetsUsed + pose2.numberOfTargetsUsed) / 2;
    double timestamp = (pose1.timestampSeconds + pose1.timestampSeconds) / 2;
    return new PoseEstimation(new Pose3d(new Pose2d(x, y, new Rotation2d(rot))), timestamp, numberOfTargets);
  }
}
