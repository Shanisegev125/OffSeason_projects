package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.Constructors.CamConfig.CamGain;

public interface AprilTagIO {

    default void updateInputs(AprilTagIOInputs inputs) {}

    class AprilTagIOInputs {
        public boolean connected = false;

        public boolean observation = false;

        public Pose2d pose = null;
        public double timestamp = 0;
        public double alpha = 0;
        public double distance = 0;
    }

    static double getStDev(CamGain gain, double distance, double alpha, double velocity, double omega) {
        return Math.sqrt(
                Math.pow(gain.kDistance() * distance, 2) +
                        Math.pow(gain.kAlpha() * alpha, 2) +
                        Math.pow(gain.kVelocity() * velocity, 2) +
                        Math.pow(gain.kOmega() * omega, 2));
    }
}
