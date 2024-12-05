package frc.robot.subsystems.Drivebase;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

    default void updateInputs(GyroIOInputs inputs) {}

    default void reset(double angle) {}

    default void updateOdometryInputs() {}

    default void updateSimOutputs(GyroIOInputs inputs) {}

    @AutoLog
    class GyroIOInputs {

        public boolean isConnected = false;

        public Rotation2d yaw = new Rotation2d();

        public double yawVelocity = 0;
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

        public double accelerationX = 0;
        public double accelerationY = 0;
        public double accelerationZ = 0;
    }
}
