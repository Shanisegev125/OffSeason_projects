package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Drivebase.Drivebase;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        return instance;
    }

    public record OdometryObservation(double timestamp, SwerveDriveWheelPositions wheelPositions,
            Rotation2d gyroAngle) {
    }

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    }

    Drivebase drivebase;
    SwerveDrivePoseEstimator poseEstimator;

    SwerveDriveWheelPositions lastWheelPositions;
    Rotation2d lastGyroAngle;

    Pose2d latestPose = new Pose2d();

    public RobotState(Drivebase drivebase) {

        this.drivebase = drivebase;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.DrivebaseConstants.KINEMATICS,
                drivebase.gyroInputs.yaw,
                drivebase.getModulesPosition(),
                new Pose2d());

        lastWheelPositions = new SwerveDriveWheelPositions(drivebase.getModulesPosition());
        lastGyroAngle = drivebase.gyroInputs.yaw;

        instance = this;
    }

    public synchronized void addOdometryObservation(OdometryObservation observation) {
        lastWheelPositions = observation.wheelPositions;
        lastGyroAngle = observation.gyroAngle;
        poseEstimator.updateWithTime(observation.timestamp, lastGyroAngle, lastWheelPositions);
    }

    public synchronized void addVisionMeasurement(VisionObservation observation) {
        if (observation == null)
            return;
        poseEstimator.addVisionMeasurement(observation.visionPose, observation.timestamp, observation.stdDevs);
    }

    public synchronized void updatePose() {
        latestPose = poseEstimator.getEstimatedPosition();
    }

    public synchronized Pose2d getPose() {
        return latestPose;
    }

    public synchronized Rotation2d getAngle() {
        return latestPose.getRotation();
    }

    public ChassisSpeeds getLatestLocalChassisSpeeds() {
        return drivebase.getLocalChassisSpeeds();
    }

    public double[] getSpeeds() {
        var speeds = drivebase.getLocalChassisSpeeds();
        return new double[] {
                Math.hypot(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond),
                speeds.omegaRadiansPerSecond
        };
    }

    public synchronized void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulesPosition, Pose2d pose) {
        poseEstimator.resetPosition(gyroAngle, modulesPosition, pose);
    }

    public synchronized void resetPose(Pose2d pose) {
        resetPose(lastGyroAngle, lastWheelPositions.positions, pose);
    }

    public boolean hasCollided() {
        return this.drivebase.hasCollided();
    }
}
