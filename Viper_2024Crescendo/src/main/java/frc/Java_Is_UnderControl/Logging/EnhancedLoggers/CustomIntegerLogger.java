package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomIntegerLogger extends IntegerLogEntry {

  private static boolean isFmsMatch;

  private String name;

  public CustomIntegerLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomIntegerLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
  }

  @Override
  public void append(long value) {
    super.append(value);
    if (!CustomIntegerLogger.isFmsMatch) {
      SmartDashboard.putNumber(this.name, value);
    }
  }
}
