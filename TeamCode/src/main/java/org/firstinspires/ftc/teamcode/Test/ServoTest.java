package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class ServoTest extends OpMode {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get(Config.OUTTAKESERVO);
    }

    @Override
    public void loop() {
        servo.setPosition(gamepad1.left_stick_y);
        telemetry.addData("position", servo.getPosition());
    }
}
