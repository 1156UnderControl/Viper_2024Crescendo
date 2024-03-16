package frc.robot.subsystems.Sensors;

import java.util.function.BooleanSupplier;

public interface DigitalInput extends BooleanSupplier {
  boolean getBoolean();

  int getBinary();
}
