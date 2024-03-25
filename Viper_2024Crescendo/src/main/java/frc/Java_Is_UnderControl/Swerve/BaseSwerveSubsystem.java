package frc.Java_Is_UnderControl.Swerve;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Logging.EnhancedLoggers.CustomChassisSpeedsLogger;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public abstract class BaseSwerveSubsystem extends SubsystemBase {
  
  private SwerveDrive swerveDrive;

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  private CustomChassisSpeedsLogger targetSpeedsLogger = new CustomChassisSpeedsLogger("/SwerveSubsystem/TargetSpeeds");

  protected BaseSwerveSubsystem(File directory, SwervePathPlannerConfig config) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    this.createSwerveDriveFromYagsl(directory);
    this.setupPathPlanner(config);
  }

  protected abstract void updateLogs();

  private void createSwerveDriveFromYagsl(File directory) {
    try {
      this.swerveDrive = new SwerveParser(directory)
        .createSwerveDrive(
          SwerveConstants.MAX_VEL,
          SwerveConstants.SteeringConversionFactor,
          SwerveConstants.DriveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  private void setupPathPlanner(SwervePathPlannerConfig config) {
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotVelocity,
      this::setChassisSpeeds,
      new HolonomicPathFollowerConfig(
        config.translationPid,
        config.anglePid,
        config.maxModuleSpeed,
        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
        new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
      },
      this
    );
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void setHeadingCorrection(boolean active) {
    swerveDrive.setHeadingCorrection(active);
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  protected Pose2d getPose() {
    return swerveDrive.getPose();
  }

  protected void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
    swerveDrive.setChassisSpeeds(chassisSpeeds);
    this.targetSpeeds = chassisSpeeds;
  }

  protected ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  protected Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  @Override
  public void periodic() {
    this.updateBaseLogs();
    this.updateLogs();
  }

  private void updateBaseLogs() {
    targetSpeedsLogger.append(this.targetSpeeds);
  }
}
