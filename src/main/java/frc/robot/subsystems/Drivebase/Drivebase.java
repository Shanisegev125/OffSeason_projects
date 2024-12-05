// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivebase;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Utils.Alert;
import frc.robot.Utils.StopWatch;
import frc.robot.subsystems.Drivebase.GyroIO.GyroIOInputs;

public class Drivebase extends SubsystemBase {

    final double CYCLE_TIME = DrivebaseConstants.CYCLE_TIME;

    public static final Lock odometryLock = new ReentrantLock();

    public GyroIO gyroIO;
    public GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    ChassisSpeeds targetGlobalRobotVelocity = new ChassisSpeeds();
    ChassisSpeeds targetGlobalRobotAdjustment = new ChassisSpeeds();
    ChassisSpeeds targetVel = new ChassisSpeeds();

    StopWatch simStopWatch;

    private boolean hasCollided = false;
    private Thread collisionThread = null;

    public SwerveModule[] swerveModules;

    private Rotation2d angle = new Rotation2d();

    SwerveModuleState[] targetModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };
    SwerveModuleState[] currentModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    private final Alert gyroAlert = new Alert("Drivebase Gyro not connected", Alert.AlertType.WARNING);

    public Drivebase(SwerveModuleIO FL, SwerveModuleIO FR, SwerveModuleIO BL,
            SwerveModuleIO BR) {

        this.swerveModules = new SwerveModule[] {
                new SwerveModule(FL, "FL"),
                new SwerveModule(FR, "FR"),
                new SwerveModule(BL, "BL"),
                new SwerveModule(BR, "BR")
        };

        gyroIO = new GyroIOPigeon2(Ports.DRIVE_GYRO);
        gyroIO.updateInputs(gyroInputs);

        if (Robot.isSimulation()) {
            this.simStopWatch = new StopWatch();
            simStopWatch.start();
        }

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> swerveModules[0].getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> swerveModules[0].getVelocity(), null);

                builder.addDoubleProperty("Front Right Angle", () -> swerveModules[1].getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> swerveModules[1].getVelocity(), null);

                builder.addDoubleProperty("Back Left Angle", () -> swerveModules[2].getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> swerveModules[2].getVelocity(), null);

                builder.addDoubleProperty("Back Right Angle", () -> swerveModules[3].getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> swerveModules[3].getVelocity(), null);

                builder.addDoubleProperty("Robot Angle", () -> angle.getRadians(), null);
            }
        });
    }

    public void setLocalDrive(ChassisSpeeds localVelocities) {
        this.targetGlobalRobotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(localVelocities, angle);
    }

    public void setGlobalDrive(ChassisSpeeds globalVelocities) {
        this.targetGlobalRobotVelocity = globalVelocities;
    }

    public void setLocalAdjustment(ChassisSpeeds localVelocities) {
        this.targetGlobalRobotAdjustment = ChassisSpeeds.fromRobotRelativeSpeeds(localVelocities, angle);
    }

    public void setGlobalAdjustment(ChassisSpeeds globalVelocities) {
        this.targetGlobalRobotAdjustment = globalVelocities;
    }

    public void stop() {
        this.targetGlobalRobotVelocity = new ChassisSpeeds();
        this.targetGlobalRobotAdjustment = new ChassisSpeeds();
    }

    public ChassisSpeeds getLocalChassisSpeeds() {
        return DrivebaseConstants.KINEMATICS.toChassisSpeeds(currentModuleStates);
    }

    public ChassisSpeeds getGlobalChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getLocalChassisSpeeds(), angle);
    }

    public SwerveModulePosition[] getModulesPosition() {
        SwerveModulePosition[] currentPositions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            currentPositions[i] = swerveModules[i].getPosition();
        }
        return currentPositions;
    }

    public void updateOdometryInputs() {
        gyroIO.updateOdometryInputs();
        for (int i = 0; i < 4; i++) {
            swerveModules[i].updateOdometryInputs();
        }
    }

    private void setCollidedTrue() {

        hasCollided = true;
        Logger.recordOutput("Swerve/Collided", true);
        if (collisionThread != null && collisionThread.isAlive())
            collisionThread.interrupt();

        collisionThread = new Thread(() -> {
            try {
                TimeUnit.MILLISECONDS.sleep(DrivebaseConstants.collision_sleep);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            hasCollided = false;
            Logger.recordOutput("Swerve/Collided", false);
        });
        collisionThread.start();
    }

    public boolean hasCollided() {
        return hasCollided;
    }

    private void skew_calculation(ChassisSpeeds wantedLocalVel) {
        // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/4
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf 10.2
        var twist = new Pose2d()
                .log(
                        new Pose2d(
                                wantedLocalVel.vxMetersPerSecond * DrivebaseConstants.skew_dt,
                                wantedLocalVel.vyMetersPerSecond * DrivebaseConstants.skew_dt,
                                new Rotation2d(wantedLocalVel.omegaRadiansPerSecond *
                                        DrivebaseConstants.skew_dt)));
        wantedLocalVel.vxMetersPerSecond = twist.dx / DrivebaseConstants.skew_dt;
        wantedLocalVel.vyMetersPerSecond = twist.dy / DrivebaseConstants.skew_dt;
        wantedLocalVel.omegaRadiansPerSecond = twist.dtheta / DrivebaseConstants.skew_dt;
    }

    private void setModulesTargets(ChassisSpeeds targetLocalRobotVel) {

        targetModuleStates = DrivebaseConstants.KINEMATICS.toSwerveModuleStates(targetLocalRobotVel);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, DrivebaseConstants.maxWheelVelocity);

        for (int i = 0; i < swerveModules.length; i++) {
            currentModuleStates[i] = swerveModules[i].getState();
            targetModuleStates[i] = SwerveModuleState.optimize(targetModuleStates[i], currentModuleStates[i].angle);

            swerveModules[i].setTarget(targetModuleStates[i]);
        }

        Logger.recordOutput("Swerve/Current", currentModuleStates);
        Logger.recordOutput("Swerve/Target", targetModuleStates);
    }

    private void accelerationLimits(ChassisSpeeds wantedLocalVel) {

        var currentLocalVel = getLocalChassisSpeeds();

        var wantedLocalAccX = (wantedLocalVel.vxMetersPerSecond - currentLocalVel.vxMetersPerSecond) / CYCLE_TIME;
        var wantedLocalAccY = (wantedLocalVel.vyMetersPerSecond - currentLocalVel.vyMetersPerSecond) / CYCLE_TIME;

        var hyp = Math.hypot(wantedLocalAccX, wantedLocalAccY);
        if (hyp > DrivebaseConstants.maxAcc) {
            var scale = DrivebaseConstants.maxAcc / hyp;
            wantedLocalAccX *= scale;
            wantedLocalAccY *= scale;
        }
 
        wantedLocalVel.vxMetersPerSecond = currentLocalVel.vxMetersPerSecond + wantedLocalAccX * CYCLE_TIME;
        wantedLocalVel.vyMetersPerSecond = currentLocalVel.vyMetersPerSecond + wantedLocalAccY * CYCLE_TIME;
    }

    @Override
    public void periodic() {

        gyroIO.updateInputs(gyroInputs);
        for (int i = 0; i < swerveModules.length; i++)
            swerveModules[i].updateInputs();

        angle = RobotState.getInstance().getAngle();

        if (Math.hypot(gyroInputs.accelerationX, gyroInputs.accelerationY) > DrivebaseConstants.collisionAcc)
            setCollidedTrue();

        targetVel = targetGlobalRobotVelocity.plus(targetGlobalRobotAdjustment);
        targetVel.omegaRadiansPerSecond = -targetVel.omegaRadiansPerSecond;

        targetVel = ChassisSpeeds.fromFieldRelativeSpeeds(targetVel, angle);
        skew_calculation(targetVel);

        var moduleMaxVel = Math.hypot(targetVel.vxMetersPerSecond, targetVel.vyMetersPerSecond)
                + Math.abs(targetVel.omegaRadiansPerSecond) * DrivebaseConstants.swerveRadius;
        if (moduleMaxVel > DrivebaseConstants.maxRobotVelocity) {
            var scale = DrivebaseConstants.maxRobotVelocity / moduleMaxVel;
            targetVel.vxMetersPerSecond *= scale;
            targetVel.vyMetersPerSecond *= scale;
            targetVel.omegaRadiansPerSecond *= scale;
        }

        if (!hasCollided)
            accelerationLimits(targetVel);

        setModulesTargets(targetVel);

        gyroAlert.set(!gyroInputs.isConnected);
        Logger.processInputs("Drive/Gyro", gyroInputs);
    }

    public double[] getCancoderAngles() {
        double[] angles = new double[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            angles[i] = swerveModules[i].getCancoderAngle();
        }
        return angles;
    }

    @Override
    public void simulationPeriodic() {
        var inputs = new GyroIOInputs();
        inputs.yawVelocity = Units.radiansToDegrees(getLocalChassisSpeeds().omegaRadiansPerSecond);
        gyroIO.updateSimOutputs(inputs);
    }
}
