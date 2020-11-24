package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class ServoTest extends OpMode {
    Servo servo;

    double pos = 0;

    @Override
    public void init() {
        servo = hardwareMap.servo.get(Config.WOBBLESERVO);
    }

    @Override
    public void loop() {
        pos += gamepad1.left_stick_y / 100;
        if(gamepad1.a)
            servo.setPosition(pos);

        telemetry.addData("target position", pos);
        telemetry.addData("servo position", servo.getPosition());
    }
}
