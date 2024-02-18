package frc.robot.subsystems.Camera;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase implements ICamera {

  public static CameraSubsystem cameraInstance = null;

  private PhotonCamera camera = new PhotonCamera("photonvision");

  private PhotonPipelineResult PhotonVisionResults = camera.getLatestResult();

  public static CameraSubsystem getInstance() {
    if (cameraInstance == null) {
      cameraInstance = new CameraSubsystem();
    }
    return cameraInstance;
  }

  @Override
  public double targetPosition() {
    return Units.degreesToRadians(this.PhotonVisionResults.getBestTarget().getYaw());
  }

  @Override
  public boolean hasTarget() {
    return this.PhotonVisionResults.hasTargets();
  }



}
