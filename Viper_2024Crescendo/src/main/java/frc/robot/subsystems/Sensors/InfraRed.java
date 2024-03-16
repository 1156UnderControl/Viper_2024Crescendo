package frc.robot.subsystems.Sensors;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;

public class InfraRed implements frc.robot.subsystems.Sensors.DigitalInput {

  private DigitalInput IR;

  private boolean inverted;

  private BooleanLogEntry measurementLog;

  public InfraRed(int port, boolean invert) {
    IR = new DigitalInput(port);
    inverted = invert;
    this.measurementLog = new BooleanLogEntry(DataLogManager.getLog(), "/sensors/InfraRed/" + port + "/measurement");
  }

  @Override
  public boolean getBoolean() {
    boolean measurement = inverted ? !IR.get() : IR.get();
    this.measurementLog.append(measurement);
    return measurement;
  }

  @Override
  public int getBinary() {
    if (getBoolean()) {
      return 1;
    } else {
      return 0;
    }
  }

  @Override
  public boolean getAsBoolean() {
    return this.getBoolean();
  }
}
