package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

public MCP9808(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
}

public class MCP9808 extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    @Override
    public Manufacturer getManufacturer()
    {

        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "Adafruit MCP9808 Temperature Sensor";
    }

    public MCP9808(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);
    }
}