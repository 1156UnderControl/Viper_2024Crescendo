package frc.robot.subsystems.Camera;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase implements ICamera {

  private static CameraSubsystem cameraInstance = null;

  // Change PhotonCamera argument to the name written in the PhotonVision UI
  private PhotonCamera camera = new PhotonCamera("photonvision");

  // private var PhotonVisionResults = camera.getLatestResult();

  @Override
  public void periodic() {

  }

  private static CameraSubsystem getInstance() {
    if (cameraInstance == null) {
      cameraInstance = new CameraSubsystem();
    }
    return cameraInstance;
  }

  @Override
  public double targetPosition() {
    // periodic()
    return 22.2;
  }

  @Override
  public boolean hasTarget() {
    return false;
  }

  /*
   * @Override
   * private double targetPosition(){
   * return 22.0;
   * 
   * }
   */

}
