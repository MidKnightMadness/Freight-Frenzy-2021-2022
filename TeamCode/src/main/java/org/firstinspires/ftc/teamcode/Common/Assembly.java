package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Callable;

//base class for subassemblies to use that contains the init code
public abstract class Assembly {

    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;

    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}
