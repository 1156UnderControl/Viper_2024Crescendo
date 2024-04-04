package frc.Java_Is_UnderControl.Vision.Odometry;

import java.util.Arrays;
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

  Pose2d referencePose;

  CustomBooleanLogger isDetectingLogger;

  CustomPose2dLogger detectedPoseLogger;

  CustomDoubleLogger numberOfDetectedTagsLogger;

  public MultiCameraPoseEstimator(List<PoseEstimator> poseEstimators) {
    this.poseEstimatorsList = poseEstimators;
  }

  public Optional<PoseEstimation> getEstimatedPose(Pose2d referencePose) {
    this.referencePose = referencePose;
    for (int i = 0; i < this.poseEstimatorsList.size(); i++) {
      removeEmptyEstimators(i, poseEstimatorsList);
    }
    for (int i = 0; i < this.poseEstimatorsList.size(); i++) {
      setArray(i, poseEstimatorsList.get(i));
    }
    PoseEstimation poseEstimation;
    Arrays.sort(arrayPosesEstimations, new ComparatorPose());
    if (arrayPosesEstimations.length % 2 == 0) {
      poseEstimation = averageTwoPoseEstimations(arrayPosesEstimations[arrayPosesEstimations.length / 2],
          arrayPosesEstimations[arrayPosesEstimations.length / 2 - 1]);
    } else {
      poseEstimation = arrayPosesEstimations[arrayPosesEstimations.length / 2];
    }
    return Optional.of(poseEstimation);
  }

  private void removeEmptyEstimators(int index, List<PoseEstimator> poseEstimator) {
    Optional<PoseEstimation> pose = poseEstimator.get(index).getEstimatedPose(referencePose);
    if (pose.isEmpty()) {
      poseEstimator.remove(index);
      return;
    }
  }

  private void setArray(int index, PoseEstimator poseEstimator) {
    Optional<PoseEstimation> pose = poseEstimator.getEstimatedPose(referencePose);
    for (int i = 1; i < pose.get().numberOfTargetsUsed; i++) {
      arrayPosesEstimations[arrayPosesEstimations.length + 1] = pose.get();
    }
    arrayPosesEstimations[index] = pose.get();
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
