package frc.lib.Constructors;

public class CanDeviceID {
    private final int mDeviceNumber;
    private final String mBus;

    public CanDeviceID(int deviceNumber, String bus) {
        mDeviceNumber = deviceNumber;
        mBus = bus;
    }

    // Use the default bus name (empty string).
    public CanDeviceID(int deviceNumber) {
        this(deviceNumber, "");
    }

    public int getDeviceNumber() {
        return mDeviceNumber;
    }

    public String getBus() {
        return mBus;
    }
}
