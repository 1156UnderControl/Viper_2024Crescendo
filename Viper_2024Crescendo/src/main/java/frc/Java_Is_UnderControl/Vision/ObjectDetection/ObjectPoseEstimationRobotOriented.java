package frc.Java_Is_UnderControl.Vision.ObjectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;

public class ObjectPoseEstimationRobotOriented {
  ObjectDetection object;

  ObjectDetectionCamera camera;

  CustomDoubleLogger cameraAngleLogger;

  CustomDoubleLogger tyLogger;

  CustomDoubleLogger txLogger;

  CustomDoubleLogger txDegreesLogger;

  CustomDoubleLogger tyDegreesLogger;

  CustomDoubleLogger distanceToObjectLogger;

  CustomDoubleLogger xObjectSDistanceLogger;

  CustomDoubleLogger yObjectDistanceLogger;

  double angleObjectInMaxTy;

  double angleObjectInMaxTx;

  public ObjectPoseEstimationRobotOriented(ObjectDetection object, ObjectDetectionCamera camera,
      double angleObjectInMaxTy, double angleObjectInMaxTx) {
    this.object = object;
    this.camera = camera;
    this.angleObjectInMaxTy = angleObjectInMaxTy;
    this.angleObjectInMaxTx = angleObjectInMaxTx;
    setupLogs();
  }

  private void setupLogs() {
    cameraAngleLogger = new CustomDoubleLogger(object.objectName + " Detection/Camera Angle");
    tyLogger = new CustomDoubleLogger(object.objectName + " Detection/Raw tY Value");
    txLogger = new CustomDoubleLogger(object.objectName + " Detection/Raw tX Value");
    tyDegreesLogger = new CustomDoubleLogger(object.objectName + " Detection/tY In Degrees");
    txDegreesLogger = new CustomDoubleLogger(object.objectName + " Detection/tX In Degrees");
    distanceToObjectLogger = new CustomDoubleLogger(object.objectName + " Detection/Camera Distance To Object");
    xObjectSDistanceLogger = new CustomDoubleLogger(object.objectName + " Detection/X Object Distance To Robot");
    yObjectDistanceLogger = new CustomDoubleLogger(object.objectName + " Detection/Y Object Distance To Robot");
  }

  public Pose2d getObjectPoseFromRobot() {
    return new Pose2d(getXdistance(), getYdistance(), new Rotation2d());
  }

  public double getYdistance() {
    double calculation = (getDistanceFromObject() * Math.cos(
        Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ()) + Units.degreesToRadians(getObjectAngleX())))
        + camera.robotToCamera.getX();
    xObjectSDistanceLogger.append(calculation);
    return calculation;
  }

  public double getXdistance() {
    double calculation = (getDistanceFromObject() * Math.sin(
        Units.radiansToDegrees(camera.robotToCamera.getRotation().getZ()) + Units.degreesToRadians(getObjectAngleX())))
        + camera.robotToCamera.getY();
    yObjectDistanceLogger.append(calculation);
    return calculation;
  }

  public double getDistanceFromObject() {
    cameraAngleLogger.append(camera.getCameraAngle());
    double calculation = camera.cameraHeight
        * Math.tan(Units.degreesToRadians(getObjectAngleY()) + Units.degreesToRadians(camera.getCameraAngle()));
    distanceToObjectLogger.append(calculation);
    return calculation;
  }

  private double getObjectAngleY() {
    tyLogger.append(object.ty);
    double calculation = (object.ty * angleObjectInMaxTy) / camera.maxTy;
    tyDegreesLogger.append(calculation);
    return calculation;
  }

  private double getObjectAngleX() {
    txLogger.append(object.tx);
    double calculation = (object.tx * angleObjectInMaxTx) / camera.maxTx;
    txDegreesLogger.append(calculation);
    return calculation;
  }

}
