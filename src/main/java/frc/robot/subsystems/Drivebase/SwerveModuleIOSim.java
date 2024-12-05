package frc.robot.subsystems.Drivebase;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.Constructors.SwerveModuleConfig;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DrivebaseConstants.SwerveModuleConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {

        private TalonFX driveMotor;
        private TalonFXSimState driveMotorSim;

        private TalonFX steerMotor;
        private TalonFXSimState steerMotorSim;

        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

        private final Queue<Double> drivePositionQueue;
        private final Queue<Double> turnPositionQueue;

        public SwerveModuleIOSim(SwerveModuleConfig config) {
                driveMotor = new TalonFX(config.driveMotorID.getDeviceNumber(), config.driveMotorID.getBus());
                driveMotor.getConfigurator().apply(SwerveModuleConstants.TalonFXConfigDrive());
                driveMotorSim = driveMotor.getSimState();

                steerMotor = new TalonFX(config.steerMotorID.getDeviceNumber(), config.steerMotorID.getBus());
                steerMotor.getConfigurator().apply(SwerveModuleConstants.TalonFXConfigSteer());
                steerMotorSim = steerMotor.getSimState();

                this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor,
                                driveMotor.getPosition());

                this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerMotor,
                                steerMotor.getPosition());

                BaseStatusSignal.setUpdateFrequencyForAll(
                                Constants.PoseEstimatorConstants.ODOMETRY_CYCLE_TIME,
                                driveMotor.getPosition(),
                                steerMotor.getPosition());

        }

        @Override
        public void updateInputs(SwerveModuleIOInputs inputs) {

                inputs.driveConnected = true;
                inputs.steerConnected = true;

                driveMotorSim.addRotorPosition(driveMotor.getVelocity().getValueAsDouble() * Robot.defaultPeriodSecs);

                inputs.drivePosition = driveMotor.getPosition().getValueAsDouble();
                inputs.driveVelocity = driveMotor.getVelocity().getValueAsDouble();

                inputs.turnAngle = Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble());

                inputs.drivePositionsArray = odometryDrivePositionsMeters;
                inputs.turnPositionsArray = odometryTurnPositions;
        }

        @Override
        public void setTarget(SwerveModuleState targetState) {
                driveMotorSim.setRotorVelocity(-targetState.speedMetersPerSecond);
                steerMotorSim.setRawRotorPosition(targetState.angle.getRotations());
        }

        @Override
        public void updateOdometryInputs() {
                // In radians with consideration of gear.
                odometryDrivePositionsMeters = drivePositionQueue.stream()
                                .mapToDouble(
                                                (Double value) -> -value)
                                .toArray();

                // In Rotation2d.
                odometryTurnPositions = turnPositionQueue.stream()
                                .map(
                                                (Double value) -> Rotation2d
                                                                .fromRotations(value))
                                .toArray(Rotation2d[]::new);

                drivePositionQueue.clear();
                turnPositionQueue.clear();
        }
}
