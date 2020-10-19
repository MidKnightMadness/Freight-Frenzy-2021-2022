package org.firstinspires.ftc.teamcode.WobbleGoal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Common.Assembly;

public abstract class WobbleGoal extends Assembly {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get(Config.WOBBLEM);
        servo1 = hardwareMap.servo.get(Config.WOBBLES);
    }

    public void lift();
    public void drop();
    public void stop();
    public void open();
    public void close();
}
