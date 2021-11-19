package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TestMotor extends OpMode {
    SampleMotor test;

    @Override
    public void init() {
        test = new SampleMotor(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            test.run();
        }
    }
}
