package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class AnalogInputTest extends OpMode {
    AnalogInput sensor;

    @Override
    public void init() {
        sensor = hardwareMap.analogInput.get("sensor");
    }

    @Override
    public void loop() {
        telemetry.addData("sensor reading", sensor.getVoltage());
    }
}
