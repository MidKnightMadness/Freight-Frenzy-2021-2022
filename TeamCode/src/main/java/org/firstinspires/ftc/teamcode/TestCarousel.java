package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TestCarousel extends OpMode {
    TestCarousel carousel;

    public TestCarousel(HardwareMap hardwareMap) {
    }

    @Override
    public void init() {
        carousel = new TestCarousel(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            //carousel.spinOn();
        }
    }
}
