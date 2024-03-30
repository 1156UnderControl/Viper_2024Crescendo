package frc.Java_Is_UnderControl.Swerve;

import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class OdometryEnabledSwerveSubsystem extends BaseSwerveSubsystem {

  public OdometryEnabledSwerveSubsystem(File directory, BaseSwerveConfig config) {
    super(directory, config);
  }

  protected double getDistanceToPosition(Translation2d targetPosition) {
    return this.getPose().getTranslation().getDistance(targetPosition);
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition) {
    this.driveAimingAtPosition(targetSpeeds, targetPosition, Rotation2d.fromDegrees(0));
  }

  protected void driveAimingAtPosition(ChassisSpeeds targetSpeeds, Translation2d targetPosition, Rotation2d offsetAngle) {
    Translation2d positionDifference = this.getPose().getTranslation().minus(targetPosition);
    Rotation2d targetAngle = positionDifference.getAngle().plus(offsetAngle);
    this.driveFieldOrientedLockedAngle(targetSpeeds, targetAngle);
  }
   
}