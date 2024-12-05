package frc.robot.subsystems.Drivebase;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Constructors.CanDeviceID;
import frc.robot.Robot;
import frc.robot.Constants.PoseEstimatorConstants;

public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 pigeon2;
    private Pigeon2SimState pigeon2SimState = null;

    Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    Queue<Double> yawPositionQueue;

    StatusSignal<Double> yawSignal;
    StatusSignal<Double> yawVelocitySignal;

    public GyroIOPigeon2(CanDeviceID config) {

        this.pigeon2 = new Pigeon2(config.getDeviceNumber(), config.getBus());
        if (Robot.isSimulation())
            pigeon2SimState = pigeon2.getSimState();

        yawSignal = pigeon2.getYaw();
        yawVelocitySignal = pigeon2.getAngularVelocityZDevice();
        BaseStatusSignal.setUpdateFrequencyForAll(PoseEstimatorConstants.ODOMETRY_CYCLE_TIME, yawSignal, yawVelocitySignal);

        pigeon2.optimizeBusUtilization();

        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon2, yawSignal);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        inputs.isConnected = BaseStatusSignal.refreshAll(yawSignal, yawVelocitySignal).isOK();

        inputs.yaw = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yawSignal, yawVelocitySignal));

        inputs.yawVelocity = pigeon2.getAngularVelocityZDevice().getValueAsDouble();

        inputs.odometryYawPositions = odometryYawPositions;

        inputs.accelerationX = pigeon2.getAccelerationX().getValueAsDouble();
        inputs.accelerationY = pigeon2.getAccelerationY().getValueAsDouble();
        inputs.accelerationZ = pigeon2.getAccelerationZ().getValueAsDouble();
    }

    @Override
    public void reset(double angle) {
        pigeon2.setYaw(angle);
    }

    @Override
    public void updateOdometryInputs() {
        odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawPositionQueue.clear();
    }

    @Override
    public void updateSimOutputs(GyroIOInputs inputs) {
        pigeon2SimState.addYaw(inputs.yawVelocity * Robot.defaultPeriodSecs);
    }
}
