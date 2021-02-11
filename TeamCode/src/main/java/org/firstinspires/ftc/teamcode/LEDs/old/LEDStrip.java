package org.firstinspires.ftc.teamcode.LEDs.old;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

//@I2cDeviceType // Only have enabled when building from Android Studio. Keep commented when building in OnBodJava.
//@DeviceProperties(name = "LED Strip", xmlTag = "LEDStrip")
public class LEDStrip extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private I2cDeviceSynch device;
    public LEDStrip(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        device = i2cDeviceSynch;
        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0x50)); // set the I2C Address
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public I2cDeviceSynch getDevice() {
        return device;
    }


    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "Adafruit LEDs";
    }
}
