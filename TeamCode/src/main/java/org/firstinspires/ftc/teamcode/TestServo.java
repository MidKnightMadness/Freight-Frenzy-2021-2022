package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TestServo extends OpMode {
    private double i;
    private Servo headServo;

    public void init() {
        headServo = hardwareMap.get(Servo.class, "head");
        i = 0;
    }

    public void loop() {
        if(gamepad1.dpad_up)
            i+=0.01;
        if(gamepad1.dpad_down)
            i-=0.01;

        headServo.setPosition(i);
        telemetry.addData("Head Servo", headServo.getPosition());
        telemetry.addData("i", i);
        telemetry.update();
    }
}
