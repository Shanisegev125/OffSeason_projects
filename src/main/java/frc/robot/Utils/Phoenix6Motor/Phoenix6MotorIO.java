package frc.robot.Utils.Phoenix6Motor;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.Constructors.CanDeviceID;

public interface Phoenix6MotorIO {

    double ratio = 1;

    default double getPosition() {
        return 0;
    }

    default double getVelocity() {
        return 0;
    }

    default double getCurrent() {
        return 0;
    }

    default double getVoltage() {
        return 0;
    }

    default void setVelocity(double velocity) {
    }

    default void setAcceleration(double acceleration) {
    }

    default void setPosition(double position) {
    }

    default void setVoltage(double voltage) {
    }

    default void addPosition(double position) {
    }

    default void stop() {
    }

    default void reset(double angle) {
    }

    default void optimizeBusUtilization() {
    }

    default void setRatio(double ratio) {
    }

    default TalonFX getMotor() {
        return null;
    }

    default CanDeviceID getCanDeviceId() {
        return new CanDeviceID(-1);
    }

    class MotorIOInputs {

        // Inputs
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double voltage = 0;
        public double temperature = 0;

        // Outputs
        public double ratio = 1;
        public double targetPosition = 0;
        public double targetVelocity = 0;
    }

}
