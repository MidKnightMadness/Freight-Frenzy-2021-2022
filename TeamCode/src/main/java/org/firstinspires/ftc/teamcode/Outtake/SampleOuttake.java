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
        motor = hardwareMap.dcMotor.get(Config.OUTTAKEM);
        servo = hardwareMap.servo.get(Config.OUTTAKES);
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

    @Override
    public void feed() {
        servo.setPosition(0.8);                     //open/feeding position
    }

    @Override
    public void resetFeed()
    {
        servo.setPosition(0.2);                      //closed position
    }
}