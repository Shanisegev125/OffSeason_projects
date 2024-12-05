package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constructors.CamConfig;
import frc.robot.Constants.VisionConstants.Note;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class NoteCam extends SubsystemBase {

  PhotonCamera cam;

  Transform3d robotToCam;
  int camWidth;
  int camHeight;

  Translation3d note;

  Matrix<N3, N3> CAMMATRIX;

  public NoteCam(CamConfig config) {
    this.cam = new PhotonCamera(config.name);
    this.camWidth = config.width;
    this.camHeight = config.height;
    this.robotToCam = config.robotToCam;
  }

  public Translation2d getBestNote() {
    var result = cam.getLatestResult();
    if (!result.hasTargets())
      return null;

    var camMatrix = cam.getCameraMatrix();
    if (camMatrix.isEmpty())
      return getBestNoteByHeight(result.getBestTarget());
    this.CAMMATRIX = camMatrix.get();

    note = getBestNoteByWidth(result.getBestTarget());
    if (note == null)
      return getBestNoteByHeight(result.getBestTarget());
    return convertToRobot2d(note);
  }

  private Translation3d getBestNoteByWidth(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();

    var minX = corners.stream().mapToDouble(corner -> corner.x).min().getAsDouble();
    var maxX = corners.stream().mapToDouble(corner -> corner.x).max().getAsDouble();
    var minY = corners.stream().mapToDouble(corner -> corner.y).min().getAsDouble();
    var maxY = corners.stream().mapToDouble(corner -> corner.y).max().getAsDouble();

    if (minX < 20 || maxX > camWidth - 20 ||
        minY < 20 || maxY > camHeight - 20) {
      return null;
    }

    double width;
    if (corners.get(0).x != corners.get(1).x)
      width = Math.abs(corners.get(0).x - corners.get(1).x);
    else
      width = Math.abs(corners.get(0).x - corners.get(2).x);

    double distance = (Note.OUTER_DIAMETER * CAMMATRIX.get(0, 0)) / width;

    return new Translation3d(distance
        * (corners.stream().mapToDouble(corner -> corner.x).average().getAsDouble() - CAMMATRIX.get(0, 2))
        / CAMMATRIX.get(0, 0),
        distance
            * (corners.stream().mapToDouble(corner -> corner.y).average().getAsDouble() - CAMMATRIX.get(1, 2))
            / CAMMATRIX.get(1, 1) * -1,
        distance);
  }

  private Translation2d getBestNoteByHeight(PhotonTrackedTarget target) {
    var x = (robotToCam.getZ() - Note.THICKNESS / 2) / Math.tan(target.getPitch() + robotToCam.getRotation().getX());
    return new Translation2d(x, x / Math.tan(target.getYaw()));
  }

  private Translation2d convertToRobot2d(Translation3d note) {
    return new Pose3d(note, new Rotation3d()).plus(robotToCam.inverse()).getTranslation().toTranslation2d();
  }

  public boolean isConnected() {
    return cam.isConnected();
  }
}
