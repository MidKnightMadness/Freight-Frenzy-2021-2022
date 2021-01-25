package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Common.Config;

import static java.lang.Math.*;

@TeleOp
public class ServoTest extends OpMode {
    CRServo servo;

    double pos  = 0;
    double snap = 0.1;

    @Override
    public void init(){
        servo = hardwareMap.crservo.get(Config.INTAKERELEASE);
    }

    @Override
    public void loop(){
        pos += (gamepad1.left_stick_y)/100;
        pos = min(max(pos,-1),1);
             if(pos> .9-snap && pos< .9+snap) pos =  .9;
        else if(pos>-.9-snap && pos<-.9+snap) pos = -.9;

        if(gamepad1.a) servo.setPower(pos);

        telemetry.addData("target", pos);
        telemetry.addData("servo", servo.getPower());
        telemetry.addData("snap", snap);
    }
}