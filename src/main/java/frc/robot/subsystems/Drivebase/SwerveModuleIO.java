package frc.robot.subsystems.Drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {

    default void updateInputs(SwerveModuleIOInputs inputs) {}

    default void setTarget(SwerveModuleState targetState) {}

    default void updateOdometryInputs() {}

    default double getCancoderAngle() {return 0;}

    class SwerveModuleIOInputs {

        public boolean driveConnected = false;
        public boolean steerConnected = false;

        public double drivePosition = 0;
        public double driveVelocity = 0;

        public Rotation2d turnAngle = Rotation2d.fromDegrees(0);

        public double[] drivePositionsArray = new double[] {};
        public Rotation2d[] turnPositionsArray = new Rotation2d[] {};
    }

}
