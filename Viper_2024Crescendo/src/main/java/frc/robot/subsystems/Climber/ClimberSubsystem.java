package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;

public class ClimberSubsystem extends SubsystemBase implements IClimber {
  private static ClimberSubsystem climberInstance = null;

  private Motor climberMotor = new SparkMAXMotor(ClimberConstants.ID_climber);

  public static ClimberSubsystem getInstance() {
    if (climberInstance == null) {
      return climberInstance = new ClimberSubsystem();
    }

    return climberInstance;
  }

  private ClimberSubsystem() {

  }

  @Override
  public void release() {
    climberMotor.set(0.5);
  }

  @Override
  public void climb() {
    climberMotor.set(-0.5);
  }

  public void stop() {
    climberMotor.set(0);
  }
}
