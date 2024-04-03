package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Logging.Pose2dLogEntry;

public class CustomPose2dLogger extends Pose2dLogEntry {

  private static boolean isFmsMatch;

  private String name;

  public CustomPose2dLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomPose2dLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
  }

  @Override
  public void appendRadians(Pose2d pose) {
    super.appendRadians(pose);
    if (!CustomPose2dLogger.isFmsMatch) {
      double[] data = new double[3];
      data[0] = pose.getTranslation().getX();
      data[1] = pose.getTranslation().getY();
      data[2] = pose.getRotation().getRadians();
      SmartDashboard.putNumberArray(this.name, data);
    }
  }

  @Override
  public void appendDegrees(Pose2d pose) {
    super.appendDegrees(pose);
    if (CustomPose2dLogger.isFmsMatch) {
      double[] data = new double[3];
      data[0] = pose.getTranslation().getX();
      data[1] = pose.getTranslation().getY();
      data[2] = pose.getRotation().getDegrees();
      SmartDashboard.putNumberArray(this.name, data);
    }
  }

}
