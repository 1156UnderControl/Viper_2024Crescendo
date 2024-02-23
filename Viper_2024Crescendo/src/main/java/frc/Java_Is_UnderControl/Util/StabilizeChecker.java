package frc.Java_Is_UnderControl.Util;

public class StabilizeChecker {
  edu.wpi.first.wpilibj.Timer timer;

  public StabilizeChecker() {
    this.timer = new edu.wpi.first.wpilibj.Timer();
    timer.reset();
    timer.start();
  }

  public boolean isStableInRange(double value, double setpoint, double range) {
    if (Util.inRange(value, setpoint - range, setpoint + range)) {
      if (this.timer.get() > 0.2) {
        return true;
      } else {
        return false;
      }
    } else {
      this.timer.reset();
      this.timer.start();
      return false;
    }
  }

  public boolean stabilizedInRange(double value, double setpoint, double range, double stabilityTime) {
    if (Util.inRange(value, setpoint - range, setpoint + range)) {
      if (this.timer.get() > stabilityTime) {
        return true;
      } else {
        return false;
      }
    } else {
      this.timer.reset();
      this.timer.start();
      return false;
    }
  }
}
