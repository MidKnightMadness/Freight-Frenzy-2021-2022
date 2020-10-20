package org.firstinspires.ftc.teamcode.WobbleGoal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Common.Assembly;

public abstract class WobbleGoal extends Assembly {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public abstract void lift();
    public abstract void drop();
    public abstract void stop();
    public abstract void open();
    public abstract void close();
}
