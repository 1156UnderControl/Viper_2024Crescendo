package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Java_Is_UnderControl.Logging.ChassisSpeedsLogEntry;

public class CustomChassisSpeedsLogger extends ChassisSpeedsLogEntry {

  private static boolean isFmsMatch;

  private String name;

  public CustomChassisSpeedsLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomChassisSpeedsLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
  }

  @Override
  public void append(ChassisSpeeds chassisSpeeds) {
    super.append(chassisSpeeds);
    if (!CustomChassisSpeedsLogger.isFmsMatch) {
      double[] data = new double[3];
      data[0] = chassisSpeeds.vxMetersPerSecond;
      data[1] = chassisSpeeds.vyMetersPerSecond;
      data[2] = chassisSpeeds.omegaRadiansPerSecond;
      SmartDashboard.putNumberArray(this.name, data);
    }
  }

}
