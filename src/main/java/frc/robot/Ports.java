package frc.robot;

import frc.lib.Constructors.CanDeviceID;

public class Ports {

    public static final CanDeviceID DRIVE_GYRO = new CanDeviceID(30, "canivore");

    public static final CanDeviceID FL_DRIVE = new CanDeviceID(14, "canivore");
    public static final CanDeviceID FL_STEER = new CanDeviceID(24, "canivore");
    public static final CanDeviceID FL_CANCODER = new CanDeviceID(4, "canivore");

    public static final CanDeviceID FR_DRIVE = new CanDeviceID(11, "canivore");
    public static final CanDeviceID FR_STEER = new CanDeviceID(21, "canivore");
    public static final CanDeviceID FR_CANCODER = new CanDeviceID(1, "canivore");

    public static final CanDeviceID BL_DRIVE = new CanDeviceID(13, "canivore");
    public static final CanDeviceID BL_STEER = new CanDeviceID(23, "canivore");
    public static final CanDeviceID BL_CANCODER = new CanDeviceID(3, "canivore");

    public static final CanDeviceID BR_DRIVE = new CanDeviceID(12, "canivore");
    public static final CanDeviceID BR_STEER = new CanDeviceID(22, "canivore");
    public static final CanDeviceID BR_CANCODER = new CanDeviceID(2, "canivore");

    public static final CanDeviceID INTAKE_MOTOR = new CanDeviceID(41);
    public static final int INTAKE_BEAMBREAKER_DIO = 0;

    public static final CanDeviceID FEEDER_MOTOR = new CanDeviceID(51);
    public static final int FEEDER_BEAMBREAKER_DIO = 7;

    public static final CanDeviceID PIVOT_LEFT_MOTOR = new CanDeviceID(58);
    public static final CanDeviceID PIVOT_RIGHT_MOTOR = new CanDeviceID(59);
    public static final int PIVOT_LIMITSWITCH_DIO = 9;

    public static final CanDeviceID SHOOTER_LOWER_MOTOR = new CanDeviceID(61);
    public static final CanDeviceID SHOOTER_UPPER_MOTOR = new CanDeviceID(62);
    public static final int SHOOTER_BEAMBREAKER_DIO = 2;

    public static final CanDeviceID CLIMB_MOTOR = new CanDeviceID(11);

    public static final int LED_STRIP_PWM = 0;
}
