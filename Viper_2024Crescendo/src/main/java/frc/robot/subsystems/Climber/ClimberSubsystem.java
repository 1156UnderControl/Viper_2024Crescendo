package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomDoubleLogger;
import frc.Java_Is_UnderControl.Util.Util;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Motors.Motor;
import frc.robot.subsystems.Motors.SparkMAXMotor;

public class ClimberSubsystem extends SubsystemBase implements IClimber {
  private static ClimberSubsystem climberInstance = null;

  private double goal;

  private Motor climberMotor = new SparkMAXMotor(ClimberConstants.ID_climber);

  private CustomDoubleLogger getPositionLogger;

  public static ClimberSubsystem getInstance() {
    if (climberInstance == null) {
      return climberInstance = new ClimberSubsystem();
    }

    return climberInstance;
  }

  private ClimberSubsystem() {
    getPositionLogger = new CustomDoubleLogger("ClimberSubsystem/Position");
    climberMotor.setPosition(0);
    climberMotor.setMotorBrake(true);
    goal = 0;
  }

  @Override
  public void periodic() {
    getPositionLogger.append(climberMotor.getPosition());
  }

  @Override
  public void release() {
    goal = 133;
    if (isAtSetpoint()) {
      climberMotor.set(0);
    } else {
      climberMotor.set(1);
    }
  }

  @Override
  public void climb() {
    goal = 0;
    if (isAtSetpoint()) {
      climberMotor.set(0);
    } else {
      climberMotor.set(-1);
    }
  }

  @Override
  public void stop() {
    climberMotor.set(0);
  }

  @Override
  public boolean isNearLimit() {
    return climberMotor.getPosition() < 5;
  }

  @Override
  public boolean isAtSetpoint() {
    return Util.inRange(climberMotor.getPosition(), goal - 3, goal + 3);
  }
}
