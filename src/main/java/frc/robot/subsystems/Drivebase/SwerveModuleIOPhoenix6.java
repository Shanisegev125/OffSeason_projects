package frc.robot.subsystems.Drivebase;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.Constructors.SwerveModuleConfig;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants.SwerveModuleConstants;
import frc.robot.Utils.tools;

public class SwerveModuleIOPhoenix6 implements SwerveModuleIO {

	private Rotation2d relative_offset;

	private final TalonFX driveMotor;
	private final TalonFX steerMotor;
	private final CANcoder steerEncoder;

	public double[] odometryDrivePositionsMeters = new double[] {};
	public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

	private final StatusSignal<Double> drivePositionSignal;
	private final StatusSignal<Double> driveVelocitySignal;

	private final StatusSignal<Double> steerPositionSignal;

	private final Queue<Double> drivePositionQueue;
	private final Queue<Double> turnPositionQueue;

	VelocityVoltage drive_request = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
	PositionVoltage steer_request = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

	public SwerveModuleIOPhoenix6(SwerveModuleConfig config) {

		this.driveMotor = new TalonFX(config.driveMotorID.getDeviceNumber(), config.driveMotorID.getBus());
		this.driveMotor.getConfigurator()
				.apply(Constants.DrivebaseConstants.SwerveModuleConstants.TalonFXConfigDrive());
		this.driveMotor.optimizeBusUtilization();

		this.steerMotor = new TalonFX(config.steerMotorID.getDeviceNumber(), config.steerMotorID.getBus());
		this.steerMotor.getConfigurator()
				.apply(Constants.DrivebaseConstants.SwerveModuleConstants.TalonFXConfigSteer());
		this.steerMotor.optimizeBusUtilization();

		this.steerEncoder = new CANcoder(config.cancoderID.getDeviceNumber(), config.cancoderID.getBus());
		this.steerEncoder.getConfigurator()
				.apply(Constants.DrivebaseConstants.SwerveModuleConstants.canCoderConfig(-config.encoderOffset));

		relative_offset = Rotation2d.fromRotations(
				MathUtil.inputModulus(-this.steerEncoder.getAbsolutePosition().getValueAsDouble()
						- this.steerMotor.getPosition().getValueAsDouble() / SwerveModuleConstants.STEER_GEAR_RATIO,
						-0.5, 0.5));

		this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor,
				driveMotor.getPosition());

		this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerMotor,
				steerMotor.getPosition());

		drivePositionSignal = driveMotor.getPosition();
		driveVelocitySignal = driveMotor.getVelocity();

		steerPositionSignal = steerMotor.getPosition();

		BaseStatusSignal.setUpdateFrequencyForAll(
				Constants.PoseEstimatorConstants.ODOMETRY_CYCLE_TIME,
				drivePositionSignal,
				steerPositionSignal);
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {

		inputs.driveConnected = BaseStatusSignal.refreshAll(drivePositionSignal, driveVelocitySignal).isOK();
		inputs.steerConnected = BaseStatusSignal.refreshAll(steerPositionSignal).isOK();

		inputs.drivePosition = drivePositionSignal.getValueAsDouble() / SwerveModuleConstants.DRIVE_RATIO;
		inputs.driveVelocity = driveVelocitySignal.getValueAsDouble() / SwerveModuleConstants.DRIVE_RATIO;

		inputs.turnAngle = Rotation2d
				.fromRotations(steerPositionSignal.getValueAsDouble() / SwerveModuleConstants.STEER_GEAR_RATIO)
				.plus(relative_offset).unaryMinus();

		inputs.drivePositionsArray = odometryDrivePositionsMeters;
		inputs.turnPositionsArray = odometryTurnPositions;
	}

	@Override
	public void setTarget(SwerveModuleState targetState) {

		var currentAngleRotations = steerPositionSignal.getValueAsDouble() / SwerveModuleConstants.STEER_GEAR_RATIO
				+ relative_offset.getRotations();

		driveMotor.setControl(
				drive_request.withVelocity(targetState.speedMetersPerSecond * SwerveModuleConstants.DRIVE_RATIO));
		steerMotor.setControl(steer_request.withPosition(
				(currentAngleRotations
						+ tools.getRotationError(-targetState.angle.getRotations(), currentAngleRotations)
						- relative_offset.getRotations())
						* SwerveModuleConstants.STEER_GEAR_RATIO));

	}

	@Override
	public void updateOdometryInputs() {
		// In radians with consideration of gear.
		odometryDrivePositionsMeters = drivePositionQueue.stream()
				.mapToDouble(
						(Double value) -> value
								* SwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS
								/ SwerveModuleConstants.DRIVE_GEAR_RATIO)
				.toArray();

		// In Rotation2d.
		odometryTurnPositions = turnPositionQueue.stream()
				.map(
						(Double value) -> Rotation2d
								.fromRotations(value
										/ SwerveModuleConstants.STEER_GEAR_RATIO)
								.plus(relative_offset).unaryMinus())
				.toArray(Rotation2d[]::new);

		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public double getCancoderAngle() {
		return steerEncoder.getAbsolutePosition().getValueAsDouble();
	}
}
