package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Callable;

//base class for subassemblies to use that contains the init code
public abstract class Assembly {

    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }
}
