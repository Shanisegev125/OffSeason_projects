package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.Constructors.CamConfig;

public class AprilTagIOLimelight implements AprilTagIO {

    private NetworkTable table = null;

    private final double[] empty = new double[0];

    Matrix<N3, N1> currentStDevs = VecBuilder.fill(0.0, 0.0, 0.0);

    public AprilTagIOLimelight(CamConfig config) {
        this.table = NetworkTableInstance.getDefault().getTable(config.name);
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        inputs.connected = table.getEntry("botpose_wpiblue").exists();
        inputs.observation = true;

        if (table.getEntry("tv").getInteger(0) == 0){
            inputs.observation = false;
            return;
        }

        var array = table.getEntry("botpose_wpiblue").getDoubleArray(empty);
        if (array.length == 0) {
            inputs.observation = false;
            return;
        }

        var delta = table.getEntry("targetpose_cameraspace").getDoubleArray(empty);
        if (delta.length == 0) {
            inputs.observation = false;
            return;
        }

        inputs.pose = new Pose2d(array[0], array[1], Rotation2d.fromDegrees(array[5]));
        inputs.timestamp = Timer.getFPGATimestamp() - (array[6] / 1000.0);
        inputs.distance = Math.sqrt(Math.pow(delta[0], 2) + Math.pow(delta[1], 2) + Math.pow(delta[2], 2));
    }
}
