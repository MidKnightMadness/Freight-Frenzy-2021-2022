package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Outtake.Outtake;

import java.util.concurrent.Callable;

public class SampleOuttake extends Outtake {

    //declare intake motor
    DcMotor motor;
    //declare feed servo
    Servo servo;

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get(Config.OUTTAKEMOTOR);
        servo = hardwareMap.servo.get(Config.OUTTAKESERVO);
    }

    //start motor
    @Override
    public void start() {
        motor.setPower(0.5);
    }

    //stop motor
    @Override
    public void stop() {
        motor.setPower(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) {
        motor.setPower(speed);
    }

    //open the ring-loader
    @Override
    public void feedRun() {
        servo.setPosition(0.8); //open feeding position
    }

    //close the ring-loader
    @Override
    public void resetFeed() {
        servo.setPosition(0.2); //closed position
    }

    //Calculate launch velocity, in feet per second
    public static double getLaunchVelocity(double x, double y, double z) {
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
                        return v0; //Multiply by some constant to instead output RPM
                    } else {
                        bMin = v0;
                    }
                }
            }
        }
        return 0.; //REPORT ERROR HERE IF LINE IS REACHED
    }

    //loads a ring into the launcher
    @Override
    public void feed() {
        feedRun();
        try { wait(2000); } //Try-catch exception may not work -- Test
        catch (InterruptedException exception) { }
        resetFeed();
        try { wait(2000); }
        catch (InterruptedException exception) { }
    }
}