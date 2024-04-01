package frc.Java_Is_UnderControl.Swerve;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.Java_Is_UnderControl.Control.PidConfig;

public class OdometryEnabledSwerveConfig extends BaseSwerveConfig {

  public final PidConfig moveToPosePidConfig;

  public OdometryEnabledSwerveConfig(double maxTranslationSpeed, double maxRotationSpeed, Alliance defaultAlliance,
      SwervePathPlannerConfig pathPlannerConfig, PidConfig moveToPosePidConfig) {
    super(maxTranslationSpeed, maxRotationSpeed, defaultAlliance, pathPlannerConfig);
    this.moveToPosePidConfig = moveToPosePidConfig;
  }
  
}
