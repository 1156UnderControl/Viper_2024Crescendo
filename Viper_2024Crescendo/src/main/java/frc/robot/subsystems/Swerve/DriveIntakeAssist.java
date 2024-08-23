package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public Translation2d getCorrectedDesiredSpeed(Pose2d robotPose, Pose2d objectPose, Translation2d robotVelocity) {
    Translation2d correctedReferencePose = CoordinatesTransform.fromRobotRelativeToF(robotPose, objectPose)
        .getTranslation();
    Translation2d desiredTranslationMovement = objectPose.getTranslation().minus(correctedReferencePose);
    return desiredTranslationMovement;
  }

}
