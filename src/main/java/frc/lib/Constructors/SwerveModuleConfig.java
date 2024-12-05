package frc.lib.Constructors;

public class SwerveModuleConfig {

    public final CanDeviceID driveMotorID;
    public final CanDeviceID steerMotorID;
    public final CanDeviceID cancoderID;

    public final double encoderOffset;

    public SwerveModuleConfig(CanDeviceID driveMotorID, CanDeviceID steerMotorID, CanDeviceID cancoderID, double encoderAngleOffset) {
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.cancoderID = cancoderID;
        this.encoderOffset = encoderAngleOffset;
    }
}
