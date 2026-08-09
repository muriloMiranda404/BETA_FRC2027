package frc.frc_java9485.motors.ctre.phoenix6;

import com.ctre.phoenix6.CANBus;


public class CanDeviceId {


    public static final String RIO_BUS = "rio";

    private final int deviceNumber;
    private final CANBus bus;


    public CanDeviceId(int deviceNumber, String bus) {
        this.deviceNumber = deviceNumber;
        this.bus = new CANBus(bus);
    }


    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, RIO_BUS);
    }

    public int getDeviceNumber() {
        return deviceNumber;
    }

    public String getBusName() {
        return bus.getName();
    }

    public CANBus getBus() {
        return bus;
    }

    public boolean isSameBusAs(CanDeviceId other) {
        return getBusName().equals(other.getBusName());
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof CanDeviceId)) {
            return false;
        }
        CanDeviceId o = (CanDeviceId) other;
        return o.deviceNumber == deviceNumber && isSameBusAs(o);
    }

    @Override
    public int hashCode() {
        int result = Integer.hashCode(deviceNumber);
        result = 31 * result + getBusName().hashCode();
        return result;
    }

    @Override
    public String toString() {
        return "CanDeviceId(" + deviceNumber + ", " + getBusName() + ")";
    }
}
