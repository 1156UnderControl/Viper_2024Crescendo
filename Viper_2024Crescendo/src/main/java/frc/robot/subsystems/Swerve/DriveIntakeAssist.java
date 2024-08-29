package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;

public class DriveIntakeAssist {
  Translation2d correctionReferencePosition;
  double correntionPercentage;

  DriveIntakeAssist() {
    this(new Translation2d(), 1);
  }

  DriveIntakeAssist(Translation2d correctionReferencePosition, double correctionPercentage) {
    this.correctionReferencePosition = correctionReferencePosition;
    this.correntionPercentage = correctionPercentage;
  }

  public ChassisSpeeds getCorrectedDesiredSpeed(Pose2d robotPose, Pose2d objectPose, ChassisSpeeds robotVelocity) {
    Translation2d correctedReferencePose = CoordinatesTransform.fromRobotRelativeToF(robotPose, objectPose)
        .getTranslation();
    Translation2d perfectTranslationDirection = objectPose.getTranslation().minus(correctedReferencePose);
    Rotation2d perfectTranslationAngle = perfectTranslationDirection.getAngle();
    Rotation2d requestedTranslationAngle = new Translation2d(robotVelocity.vxMetersPerSecond,
        robotVelocity.vyMetersPerSecond).getAngle();
    Rotation2d requestedToPerfectAngleDifference = perfectTranslationAngle.minus(requestedTranslationAngle);
    Rotation2d desiredTranslationAngle = requestedTranslationAngle
        .plus(requestedToPerfectAngleDifference.times(correntionPercentage));
    double robotVelocityIntensity = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    double desiredVx = robotVelocityIntensity * desiredTranslationAngle.getCos();
    double desiredVy = robotVelocityIntensity * desiredTranslationAngle.getSin();
    return new ChassisSpeeds(desiredVx, desiredVy, robotVelocity.omegaRadiansPerSecond);
  }

}
