package frc.robot.subsystems.Vision;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constructors.CamConfig.CamGain;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.Alert;
import frc.robot.subsystems.Vision.AprilTagIO.AprilTagIOInputs;

public class Vision extends SubsystemBase {

    private static Vision instance = null;

    public static Vision getInstance() {
        if (instance == null) {
            throw new IllegalStateException("RobotState has not been initialized");
        }
        return instance;
    }

    AprilTagIOPhoton frontCam;
    AprilTagIOLimelight backCam;

    AprilTagIOInputs frontCamInputs = new AprilTagIOInputs();
    AprilTagIOInputs backCamInputs = new AprilTagIOInputs();

    ScheduledExecutorService executor;

    private final Alert frontCamDisconnectedAlert = new Alert("Front cam disconnected!", Alert.AlertType.WARNING);
    private final Alert backDisconnectedAlert = new Alert("Back cam disconnected!", Alert.AlertType.WARNING);

    public Vision() {
        frontCam = new AprilTagIOPhoton(VisionConstants.FRONTCAM, VisionConstants.aprilTagFieldLayout);
        // backCam = new AprilTagIOLimelight(VisionConstants.BACKCAM);

        executor = Executors.newScheduledThreadPool(2);

        executor.scheduleAtFixedRate(
                () -> updateAprilTag(frontCam, frontCamInputs, RobotState.getInstance(), VisionConstants.FRONTCAM.gains,
                        VisionConstants.FRONTCAM.rotGains),
                0, (long) (1000 / VisionConstants.FRONTCAM_CYCLE_TIME), TimeUnit.MILLISECONDS);
        // executor.scheduleAtFixedRate(
        //         () -> updateAprilTag(backCam, backCamInputs, RobotState.getInstance(), VisionConstants.BACKCAM.gains,
        //                 VisionConstants.BACKCAM.rotGains),
        //         0, (long) (1000 / VisionConstants.BACKCAM_CYCLE_TIME), TimeUnit.MILLISECONDS);

        instance = this;
    }

    @Override
    public void periodic() {

        frontCamDisconnectedAlert.set(!frontCamInputs.connected);
        backDisconnectedAlert.set(!backCamInputs.connected);

        // Logger.processInputs("Vision/frontCam", frontCamInputs);
        // Logger.processInputs("Vision/backCam", backCamInputs);
    }

    private static void updateAprilTag(AprilTagIO cam, AprilTagIOInputs inputs, RobotState robotState,
            CamGain gains, CamGain rotGains) {
        var speeds = robotState.getSpeeds();
        cam.updateInputs(inputs);

        if (!inputs.observation)
            return;

        var posStDev = AprilTagIO.getStDev(gains, inputs.distance, inputs.alpha, speeds[0], speeds[1]);
        var rotStDev = AprilTagIO.getStDev(rotGains, inputs.distance, inputs.alpha, speeds[0], speeds[1]);

        if (posStDev < VisionConstants.ResetStDevThreshhold) {
            robotState.resetPose(inputs.pose);
            return;
        }

        robotState.addVisionMeasurement(
                new VisionObservation(inputs.pose, inputs.timestamp, VecBuilder.fill(posStDev, posStDev, rotStDev)));
    }

    public Pose2d getFrontCamPose() {
        return frontCamInputs.observation ? frontCamInputs.pose : null;
    }

    public Pose2d getBackCamPose() {
        return backCamInputs.observation ? backCamInputs.pose : null;
    }

    public AprilTagIOPhoton getFrontCam() {
        return frontCam;
    }

    public AprilTagIOLimelight getBackCam() {
        return backCam;
    }

    public boolean isFrontCamConnected() {
        return frontCamInputs.connected;
    }

    public boolean isBackCamConnected() {
        return backCamInputs.connected;
    }

}
