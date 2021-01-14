package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleOuttake extends Outtake {

    //declare intake motor
    DcMotorEx motor;
    private int targetVel;
    private double lastVel;

    //declare feed servo
    Servo servo;

    //tower goal position
    private double towerX = -28.75;  //TODO: correct tower position –– FINISHED, see below and double check
    private double towerY = 80;
    private double towerZ = 35.5;

    /*
    Phys. tower goal position (idk about actual translation over)
     x = -12.5 in
     y = the whole field minus 18 in
     z = 35.5 in
     */

    private static final int ticksPerSecPerFeetPerSec = 1000;  //TODO: get correct ft/s to t/s constant

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        super.init(hardwareMap, telemetry, gamepad1, gamepad2);
        motor = hardwareMap.get(DcMotorEx.class, Config.OUTTAKEMOTOR);
        servo = hardwareMap.servo.get(Config.OUTTAKESERVO);
    }

    //start motor
    @Override
    public void start() {
        targetVel = 950;
        motor.setVelocity(targetVel);
        telemetry.addData("outtake velocity", motor.getVelocity());
    }

    @Override
    public void startFromPos(double x, double y, double z) {
        targetVel = (int)getLaunchVelocity(x - towerX, y - towerY, z - towerZ);
        motor.setVelocity(targetVel);
        telemetry.addData("outtake velocity", motor.getVelocity());
    }

    //stop motor
    @Override
    public void stop() {
        targetVel = 0;
        motor.setVelocity(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) {
        motor.setPower(speed);
    }

    @Override
    //checks if outtake is the right speed for shooting
    public boolean isReady() {
        boolean ready = Math.abs((motor.getVelocity() + lastVel)*.5 - targetVel) <= 10;
        lastVel = motor.getVelocity();
        return ready;
    }

    //open the ring-loader
    @Override
    public void feedRun() {
        servo.setPosition(0.55); //open feeding position
    }

    //close the ring-loader
    @Override
    public void resetFeed() {
        servo.setPosition(0.2); //closed position
    }

    //Calculate launch velocity, in feet per second
    private static double getLaunchVelocity(double x, double y, double z) {
        double a = 30.; //Launch angle (from ground)
        double bMin = 15.; //minimum velocity (ft/s)
        double bMax = 50.; //maximum velocity (ft/s)

        for (int i = 0; i < 10000; i++) { //Maximum 10000 iters
            double g = 32.2; //gravity, feet/sec^2
            double v0 = (bMin + bMax) * .5; //velocity at mean
            double tx = Math.sqrt(x * x + y * y); //dist to goal along XY plane
            double vx = Math.cos((a * Math.PI) / 180.) * v0; //x velocity
            double vt = Math.tan((a * Math.PI) / 180.); //tangent slope
            double xvx = tx / vx; //split to optimize computation time
            double ty = -.5 * g * xvx * xvx + vt * tx; //vertical hit point
            double dev = ty - z; //how far off we are
            double mx = (vx * vx * vt) / g; //peak distance
            if (mx < tx) { //parabola peak occurs before hit
                bMin = v0;
            } else {
                if (dev / Math.abs(dev) > 0.) {
                    bMax = v0;
                } else {
                    if (Math.abs(dev) < .0000001) { //precision threshold
                        return v0 * ticksPerSecPerFeetPerSec; //Multiply by some constant to instead output RPM
                    } else {
                        bMin = v0;
                    }
                }
            }
        }
        return 0.; //REPORT ERROR HERE IF LINE IS REACHED
    }

    //Reference for checking the following function
    //https://www.desmos.com/calculator/kszz4yrkd4
    //Calculate launch angle, in degrees
    private static double getLaunchAngle(double x, double y, double z) {
        double v = 10000. / (float)ticksPerSecPerFeetPerSec;
        double bMin = 15.; //minimum angle (deg)
        double bMax = 45.; //maximum angle (deg) //Above 45 gets hairy

        for (int i = 0; i < 1000; i++) { //Maximum 1000 iters
            double g = 32.2; //gravity, feet/sec^2
            double a0 = (bMin+bMax) * .5; //angle at mean
            double tx = Math.sqrt(x*x + y*y); //dist to goal along XY plane
            double vx = Math.cos((a0*Math.PI)/180.)*v; //x velocity
            double vt = Math.tan((a0*Math.PI)/180.); //tangent slope
            double xvx = tx / vx; //split to optimize computation time
            double ty = -.5*g*xvx*xvx+vt*tx; //vertical hit point
            double dev = ty - z; //how far off we are
            double mx = (vx*vx*vt)/g; //peak distance
            if (dev / Math.abs(dev) > 0.) {
                bMax = a0; //Hit was too high, refine maximum
            } else {
                if (Math.abs(dev) < .00001) { //precision threshold
                    return a0; //Calculated angle
                } else {
                    bMin = a0; //Hit was too low, refine minimum
                }
            }
        }
        return(45.); //Target is too far for chosen velocity to hit
    }

    //loads a ring into the launcher
    @Override
    public void feed() {
        feedRun();
        try { wait(2000); } //Try-catch exception may not work -- Test
        catch (InterruptedException ignored) { }
        resetFeed();
        try { wait(2000); }
        catch (InterruptedException ignored) { }
    }
}