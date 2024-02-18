package frc.Java_Is_UnderControl.PhotonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;

public class PhotonVision {

  static private PhotonCamera camera = new PhotonCamera("photonvision");

  static private PhotonPipelineResult getResult() {
    return camera.getLatestResult();
  }

  static public double targetPositionX() {
    return Units.degreesToRadians(getResult().getBestTarget().getYaw());
  }

  static public double targetPositionY() {
    return getResult().getBestTarget().getPitch();
  }

  static public boolean hasTarget() {
    return getResult().hasTargets();
  }

}
