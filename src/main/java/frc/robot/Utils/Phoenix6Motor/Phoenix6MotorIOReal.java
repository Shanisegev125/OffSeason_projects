package frc.robot.Utils.Phoenix6Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.Constructors.CanDeviceID;

public class Phoenix6MotorIOReal implements Phoenix6MotorIO {

    private TalonFX motor;

    double ratio = 1;

    VelocityVoltage velocity_request = new VelocityVoltage(0).withSlot(0).withAcceleration(0).withEnableFOC(false);
    PositionVoltage position_request = new PositionVoltage(0).withSlot(0).withVelocity(0);

    public Phoenix6MotorIOReal(CanDeviceID id) {
        motor = new TalonFX(id.getDeviceNumber(), id.getBus());
    }

    public Phoenix6MotorIOReal(CanDeviceID id, TalonFXConfiguration config) {
        motor = new TalonFX(id.getDeviceNumber(), id.getBus());
        motor.getConfigurator().apply(config);
    }

    public TalonFXConfigurator getConfigurator() {
        return motor.getConfigurator();
    }

    public void setRatio(double ratio) {
        // a value of 2 means that for every 2 turns of the motor, the output shaft turns once
        this.ratio = ratio;
    }

    public void optimizeBusUtilization() {
        motor.optimizeBusUtilization();
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
        motor.setControl(velocity_request.withVelocity(velocity * ratio));
    }

    public void setAcceleration(double acceleration) {
        motor.setControl(velocity_request.withAcceleration(acceleration * ratio));
    }

    public void setPosition(double position) {
        motor.setControl(position_request.withPosition(position * ratio));
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void addPosition(double position) {
        setPosition((position + getPosition()) * ratio);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void reset(double angle) {
        motor.setPosition(angle * ratio);
    }

    public TalonFX getMotor() {
        return motor;
    }

    public CanDeviceID getCanDeviceId() {
        return new CanDeviceID(motor.getDeviceID());
    }

    public void updateInputs(MotorIOInputs inputs) {
        inputs.position = getPosition();
        inputs.velocity = getVelocity(); 
        inputs.current = motor.getSupplyCurrent().getValueAsDouble();
        inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.temperature = motor.getDeviceTemp().getValueAsDouble();

        if (inputs.targetPosition != 0) {
            setPosition(inputs.targetPosition);
        }
        if (inputs.targetVelocity != 0) {
            setVelocity(inputs.targetVelocity);
        }
    }
}
