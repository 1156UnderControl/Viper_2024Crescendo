package frc.Java_Is_UnderControl.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class CoordinatesTransform {
  public static Pose2d fromRobotRelativeToF(Pose2d robotPose, Pose2d poseForConversion) {
    // double xPoseInField = (poseForConversion.getX() *
    // robotPose.getRotation().getCos()
    // + robotPose.getX()) + poseForConversion.getY() *
    // robotPose.getRotation().getSin();
    // double yPoseInField = (poseForConversion.getY() *
    // robotPose.getRotation().getSin()
    // + robotPose.getY()) + poseForConversion.getY() *
    // robotPose.getRotation().getCos();
    // Pose2d poseInTheField = new Pose2d(xPoseInField, yPoseInField, new
    // Rotation2d());
    Pose2d poseInTheField = robotPose
        .plus(new Transform2d(poseForConversion.getTranslation(), poseForConversion.getRotation()));
    return poseInTheField;
  }
}
