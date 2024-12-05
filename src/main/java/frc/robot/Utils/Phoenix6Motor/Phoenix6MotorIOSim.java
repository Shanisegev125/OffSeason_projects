package frc.robot.Utils.Phoenix6Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.lib.Constructors.CanDeviceID;

public class Phoenix6MotorIOSim implements Phoenix6MotorIO {

    private TalonFX motor;
    private TalonFXSimState simMotor;

    double ratio = 1;

    public Phoenix6MotorIOSim(CanDeviceID id) {
        motor = new TalonFX(id.getDeviceNumber(), id.getBus());
        simMotor = motor.getSimState();
    }

    public Phoenix6MotorIOSim(CanDeviceID id, TalonFXConfiguration config) {
        motor = new TalonFX(id.getDeviceNumber(), id.getBus());
        motor.getConfigurator().apply(config);
        simMotor = motor.getSimState();
    }

    public void setRatio(double ratio) {
        // a value of 2 means that for every turn of the motor, the output shaft turns twice
        this.ratio = ratio;
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble() / ratio;
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble() / ratio;
    }

    public double getCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public void setVelocity(double velocity) {
        simMotor.setRotorVelocity(velocity * ratio);
    }

    public void setAcceleration(double acceleration) {
        simMotor.setRotorAcceleration(acceleration * ratio);
    }

    public void setPosition(double position) {
        simMotor.setRawRotorPosition(position * ratio);
    }

    public void addPosition(double position) {
        simMotor.addRotorPosition(position * ratio);
    }

    public void stop() {
        setPosition(0);
        setVelocity(0);
        setAcceleration(0);
    }

    public void reset(double angle) {
        setPosition(angle * ratio);
    }

    public TalonFX getMotor() {
        return motor;
    }

    public void updateInputs(MotorIOInputs inputs) {
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
        inputs.current = -1;
        inputs.voltage = -1;
        inputs.temperature = -1;

        if (inputs.targetPosition != 0) {
            setPosition(inputs.targetPosition);
        }
        if (inputs.targetVelocity != 0) {
            setVelocity(inputs.targetVelocity);
        }
    }

}
