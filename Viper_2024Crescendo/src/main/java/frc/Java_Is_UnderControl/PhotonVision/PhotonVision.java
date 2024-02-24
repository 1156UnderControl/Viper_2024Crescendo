package frc.Java_Is_UnderControl.PhotonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class PhotonVision {

  private PhotonCamera camera;

  private BooleanLogEntry hasTargetLogEntry;

  private DoubleLogEntry targetPositionXLogEntry;

  private DoubleLogEntry targetPositionYLogEntry;

  public PhotonVision() {
    this("Microsoft_LifeCam_HD-3000");
  }

  public PhotonVision(String cameraName) {
    this.camera = new PhotonCamera(cameraName);
    this.hasTargetLogEntry = new BooleanLogEntry(DataLogManager.getLog(), "/Cameras/" + cameraName + "/hasTarget");
    this.targetPositionXLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
        "/Cameras/" + cameraName + "/targetPositionX");
    this.targetPositionYLogEntry = new DoubleLogEntry(DataLogManager.getLog(),
        "/Cameras/" + cameraName + "/targetPositionY");
  }

  private PhotonPipelineResult getResult() {
    return this.camera.getLatestResult();
  }

  public double targetPositionX() {
    PhotonTrackedTarget bestTarget = this.getResult().getBestTarget();
    if (bestTarget != null) {
      double targetPositionX = Units.degreesToRadians(bestTarget.getYaw());
      this.targetPositionXLogEntry.append(targetPositionX);
      return targetPositionX;
    } else {
      return 0;
    }
  }

  public double targetPositionY() {
    double targetPositionY = this.getResult().getBestTarget().getPitch();
    this.targetPositionYLogEntry.append(targetPositionY);
    return targetPositionY;
  }

  public boolean hasTarget() {
    boolean hasTarget = this.getResult().hasTargets();
    this.hasTargetLogEntry.append(hasTarget);
    return hasTarget;
  }

}
