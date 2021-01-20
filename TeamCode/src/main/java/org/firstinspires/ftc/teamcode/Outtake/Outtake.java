package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.teamcode.Common.Assembly;

public abstract class Outtake extends Assembly {

    public abstract void start();
    public abstract void startPowerShot();
    public abstract void startFromPos(double x, double y, double z);
    public abstract void stop();
    public abstract void setSpeed(double speed);
    public abstract boolean isReady();

    public abstract void feedRun();
    public abstract void resetFeed();
    public abstract void feed();

}
