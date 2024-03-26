package frc.Java_Is_UnderControl.Swerve;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BaseSwerveConfig {

  public double maxTranslationSpeed;

  public double maxRotationSpeed;

  public Alliance defaultAlliance;

  public SwervePathPlannerConfig pathPlannerConfig;

  public BaseSwerveConfig(double maxTranslationSpeed, double maxRotationSpeed, Alliance defaultAlliance,
      SwervePathPlannerConfig pathPlannerConfig) {
    this.maxTranslationSpeed = maxTranslationSpeed;
    this.maxRotationSpeed = maxRotationSpeed;
    this.defaultAlliance = defaultAlliance;
    this.pathPlannerConfig = pathPlannerConfig;
  }
  
}
