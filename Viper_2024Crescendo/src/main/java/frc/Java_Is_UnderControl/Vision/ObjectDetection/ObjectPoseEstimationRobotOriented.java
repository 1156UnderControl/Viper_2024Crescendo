package frc.Java_Is_UnderControl.Vision.ObjectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ObjectPoseEstimationRobotOriented {
  ObjectDetection object;

  ObjectDetectionCamera camera;

  double angleObjectInMaxTy;

  double angleObjectInMaxTx;

  public ObjectPoseEstimationRobotOriented(ObjectDetection object, ObjectDetectionCamera camera,
      double angleObjectInMaxTy, double angleObjectInMaxTx) {
    this.object = object;
    this.camera = camera;
    this.angleObjectInMaxTy = angleObjectInMaxTy;
    this.angleObjectInMaxTx = angleObjectInMaxTx;
  }

  public Pose2d getObjectPoseFromRobot() {
    return new Pose2d(getXdistance(), getYdistance(), new Rotation2d());
  }

  public double getXdistance() {
    return (getDistanceFromObject() * Math.cos(
        Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ()) + getObjectAngleX()))
        + camera.robotToCamera.getX();
  }

  public double getYdistance() {
    return (getDistanceFromObject() * Math.sin(
        Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ()) + getObjectAngleX()))
        + camera.robotToCamera.getY();
  }

  public double getDistanceFromObject() {
    return camera.cameraHeight * Math.tan(getObjectAngleY() + camera.getCameraAngle());
  }

  private double getObjectAngleY() {
    return (object.ty * angleObjectInMaxTy) / camera.maxTy;
  }

  private double getObjectAngleX() {
    return (object.tx * angleObjectInMaxTx) / camera.maxTx;
  }

}
