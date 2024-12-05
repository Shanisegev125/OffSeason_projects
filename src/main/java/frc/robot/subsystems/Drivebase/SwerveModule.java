package frc.robot.subsystems.Drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Alert;
import frc.robot.subsystems.Drivebase.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModule extends SubsystemBase {

    private final SwerveModuleIO io;
    private final SwerveModuleIOInputs inputs = new SwerveModuleIOInputs();

    public String name;

    private final Alert driveMotorDisconnected;
    private final Alert steerMotorDisconnected;

    public SwerveModule(SwerveModuleIO io, String name) {
        this.io = io;
        this.name = name;

        driveMotorDisconnected = new Alert(name + " drive Motor Disconnected", Alert.AlertType.WARNING);
        steerMotorDisconnected = new Alert(name + " drive Motor Disconnected", Alert.AlertType.WARNING);

    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        // Logger.processInputs("Drive/Module" + name, inputs);

        driveMotorDisconnected.set(!inputs.driveConnected);
        steerMotorDisconnected.set(!inputs.steerConnected);

    }

    public void setTarget(SwerveModuleState targetState) {
        io.setTarget(targetState);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.driveVelocity, inputs.turnAngle);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePosition, inputs.turnAngle);
    }

    public double getVelocity() {
        return inputs.driveVelocity;
    }

    public double getRadians() {
        return inputs.turnAngle.getRadians();
    }

    public double[] getDrivePositionsArray() {
        return inputs.drivePositionsArray;
    }

    public Rotation2d[] getTurnPositionsArray() {
        return inputs.turnPositionsArray;
    }

    public void updateOdometryInputs() {
        io.updateOdometryInputs();
    }

    public double getCancoderAngle() {
        return io.getCancoderAngle();
    }
}
