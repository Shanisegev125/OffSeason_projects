package frc.robot.Utils.Phoenix6Motor;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Constructors.CanDeviceID;
import frc.robot.Constants.TesterConstants;

public class Phoenix6MotorIOTester extends Phoenix6MotorIOReal {

    private static ArrayList<Phoenix6MotorIOTester> instances = new ArrayList<Phoenix6MotorIOTester>();

    public static void createInstance(CanDeviceID id) {
        instances.add(new Phoenix6MotorIOTester(id));
    }

    public static ArrayList<Phoenix6MotorIOTester> getInstances() {
        return instances;
    }

    public Phoenix6MotorIOTester(CanDeviceID id) {
        super(id, TesterConstants.talonFXConfig());
        SmartDashboard.setDefaultNumber("Motor" + id.getDeviceNumber() + "/VelocityOut", 0);
        SmartDashboard.setDefaultNumber("Motor" + id.getDeviceNumber() + "/VoltageOut", 0);
    }

    public void update() {
        SmartDashboard.putNumber("Motor" + getCanDeviceId().getDeviceNumber() + "/Position", getPosition());
        SmartDashboard.putNumber("Motor" + getCanDeviceId().getDeviceNumber() + "/Velocity", getVelocity() * 60);
        SmartDashboard.putNumber("Motor" + getCanDeviceId().getDeviceNumber() + "/Voltage", getVoltage());
        SmartDashboard.putNumber("Motor" + getCanDeviceId().getDeviceNumber() + "/Current", getCurrent());

        var vel = SmartDashboard.getNumber("Motor" + getCanDeviceId().getDeviceNumber() + "/VelocityOut", 0) / 60;
        var volt = SmartDashboard.getNumber("Motor" + getCanDeviceId().getDeviceNumber() + "/VoltageOut", 0);

        if (vel != 0) {
            setVelocity(vel);
        } else if (volt != 0) {
            setVoltage(volt);
        } else {
            setVoltage(0);
        }
    }
}
