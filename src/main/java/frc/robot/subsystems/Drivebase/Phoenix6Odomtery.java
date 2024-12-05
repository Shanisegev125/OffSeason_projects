package frc.robot.subsystems.Drivebase;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryObservation;

public class Phoenix6Odomtery extends SubsystemBase {

        RobotState robotState;
        Drivebase drivebase;
        SwerveModulePosition[] currentModulesPositions = {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
        };

        double deltaTime = 0;
        double timestamp = 0;
        double lastTimestamp = 0;

        Rotation2d gyroAngle;

        public Phoenix6Odomtery(RobotState robotState, Drivebase drivebase) {
                this.drivebase = drivebase;
                this.robotState = robotState;
        }

    @Override
    public void periodic() {

        Drivebase.odometryLock.lock();
        drivebase.updateOdometryInputs();
        Drivebase.odometryLock.unlock();

        if (!robotState.hasCollided()) {

            int deltaCount = Math.min(drivebase.swerveModules[0].getDrivePositionsArray().length,
                    drivebase.gyroInputs.odometryYawPositions.length);

            timestamp = MathSharedStore.getTimestamp();
            deltaTime = (timestamp - lastTimestamp) / deltaCount;
            lastTimestamp = timestamp;

            for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {

                for (int i = 0; i < 4; i++) {

                    currentModulesPositions[i].distanceMeters = drivebase.swerveModules[i]
                            .getDrivePositionsArray()[deltaIndex];
                    currentModulesPositions[i].angle = drivebase.swerveModules[i]
                            .getTurnPositionsArray()[deltaIndex];
                }

                robotState.addOdometryObservation(new OdometryObservation(
                        timestamp + deltaTime * deltaIndex,
                        new SwerveDriveWheelPositions(currentModulesPositions),
                        drivebase.gyroInputs.odometryYawPositions[deltaIndex]));

            }
        } else {

            int lastIndex = Math.min(drivebase.swerveModules[0].getDrivePositionsArray().length,
                    drivebase.gyroInputs.odometryYawPositions.length) - 1;

            for (int i = 0; i < 4; i++) {
                currentModulesPositions[i].distanceMeters = drivebase.swerveModules[i]
                        .getDrivePositionsArray()[lastIndex];
                currentModulesPositions[i].angle = drivebase.swerveModules[i]
                        .getTurnPositionsArray()[lastIndex];
            }

            robotState.resetPose(
                drivebase.gyroInputs.odometryYawPositions[lastIndex],
                    currentModulesPositions,
                    robotState.getPose());
        }
    }
}
