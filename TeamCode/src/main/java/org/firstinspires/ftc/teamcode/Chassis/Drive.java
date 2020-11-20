package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Common.Assembly;

import java.util.concurrent.Callable;

public abstract class Drive extends Assembly {

    public abstract void drive(double forwards, double sideways, double turn);

    public abstract void move(double inchesX, double inchesY);
    public abstract void turn(double degrees);
    public abstract void alignForward();

    public abstract void moveToPosition(double x, double y);
    public abstract void moveToPosition(VectorF target);
    public abstract void moveToTower();

    public abstract double getAngle();

    public Callable<Boolean> isStopRequested;  //you need to set this is
    public abstract void stop();
}
