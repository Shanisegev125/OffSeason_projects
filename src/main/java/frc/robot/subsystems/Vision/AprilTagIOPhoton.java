package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Constructors.CamConfig;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class AprilTagIOPhoton implements AprilTagIO {

  private PhotonCamera cam;

  private PhotonPoseEstimator poseEstimator;

  public AprilTagIOPhoton(CamConfig config, AprilTagFieldLayout aprilTagFieldLayout) {

    this.cam = new PhotonCamera(config.name);

    this.poseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        this.cam,
        config.robotToCam);
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    inputs.connected = cam.isConnected();

    inputs.observation = true;

    var estimation = poseEstimator.update();
    if (estimation.isEmpty()) {
      inputs.observation = false;
      return;
    }

    inputs.pose = estimation.get().estimatedPose.toPose2d();
    inputs.timestamp = Timer.getFPGATimestamp() - estimation.get().timestampSeconds;
    inputs.distance = estimation.get().targetsUsed.stream().mapToDouble(target -> target.getBestCameraToTarget().getX())
        .average().getAsDouble();
    inputs.alpha = estimation.get().targetsUsed.stream()
        .mapToDouble(target -> Math.sqrt(Math.pow(target.getYaw(), 2) + Math.pow(target.getPitch(), 2))).average()
        .getAsDouble();
  }
}