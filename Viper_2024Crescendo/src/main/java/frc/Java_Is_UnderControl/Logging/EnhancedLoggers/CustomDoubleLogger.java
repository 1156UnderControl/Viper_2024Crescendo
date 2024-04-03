package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomDoubleLogger extends DoubleLogEntry {

  private static boolean isFmsMatch;

  private String name;

  public CustomDoubleLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomDoubleLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
  }

  @Override
  public void append(double value) {
    super.append(value);
    if (!CustomDoubleLogger.isFmsMatch) {
      SmartDashboard.putNumber(this.name, value);
    }
  }

}
