package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Logging.Pose3dLogEntry;

public class CustomPose3dLogger extends Pose3dLogEntry {

  private static boolean isFmsMatch;

  private String name;

  public CustomPose3dLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomPose3dLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
  }

  @Override
  public void appendRadians(Pose3d pose) {
    super.appendRadians(pose);
    if (!CustomPose3dLogger.isFmsMatch) {
      double[] data = new double[4];
      data[0] = pose.getTranslation().getX();
      data[1] = pose.getTranslation().getY();
      data[2] = pose.getZ();
      data[3] = pose.getRotation().getAngle();
      SmartDashboard.putNumberArray(this.name, data);
    }
  }

  @Override
  public void appendDegrees(Pose3d pose) {
    super.appendDegrees(pose);
    if (CustomPose3dLogger.isFmsMatch) {
      double[] data = new double[3];
      data[0] = pose.getTranslation().getX();
      data[1] = pose.getTranslation().getY();
      data[2] = Units.radiansToDegrees(pose.getRotation().getAngle());
      SmartDashboard.putNumberArray(this.name, data);
    }
  }

}
