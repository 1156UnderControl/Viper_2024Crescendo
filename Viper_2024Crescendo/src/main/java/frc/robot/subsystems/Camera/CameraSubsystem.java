package frc.robot.subsystems.Camera;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase implements ICamera {

  private static CameraSubsystem cameraInstance = null;

  // Change PhotonCamera argument to the name written in the PhotonVision UI
  private PhotonCamera camera = new PhotonCamera("photonvision");

  private PhotonPipelineResult PhotonVisionResults = camera.getLatestResult();

  public CameraSubsystem() {

  }

  public CameraSubsystem getInstance() {
    if (cameraInstance == null) {
      cameraInstance = new CameraSubsystem();
    }
    return cameraInstance;
  }

  @Override
  public double targetPosition() {
    return this.PhotonVisionResults.getBestTarget().getYaw();
  }

  @Override
  public boolean hasTarget() {
    return this.PhotonVisionResults.hasTargets();
  }

}
