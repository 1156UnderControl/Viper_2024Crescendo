package frc.Java_Is_UnderControl.Logging.EnhancedLoggers;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomBooleanLogger extends BooleanLogEntry {

  private static boolean isFmsMatch;

  private String name;

  public CustomBooleanLogger(String name) {
    super(DataLogManager.getLog(), name);
    this.name = name;
    CustomBooleanLogger.isFmsMatch = DriverStation.getMatchNumber() > 0;
  }

  @Override
  public void append(boolean value) {
    super.append(value);
    if (!CustomBooleanLogger.isFmsMatch) {
      SmartDashboard.putBoolean(this.name, value);
    }
  }

}
