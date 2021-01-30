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
    public abstract void move(double inchesX, double inchesY, double power);
    public abstract void smoothMove(double inchesX, double inchesY);
    public abstract void adjust(double x, double y);
    public abstract void turn(double degrees);
    public abstract void turnToPoint(double x, double y);
    public abstract void alignForward();

    public abstract void moveToPosition(double x, double y);
    public abstract void moveToPosition(double x, double y, double power);
    public abstract void moveToPosition(VectorF target);
    public abstract void moveToTower();
    public abstract void moveToPower1();
    public abstract void moveToPower2();
    public abstract void moveToPower3();

    public abstract double getAngle();
    public abstract double getCurrentX();
    public abstract double getCurrentY();

    public abstract void setAngle(double angle);
    public abstract void setCurrentX(double x);
    public abstract void setCurrentY(double y);

    public Callable<Boolean> isStopRequested;  //you need to set this is
    public abstract void stop();
}
