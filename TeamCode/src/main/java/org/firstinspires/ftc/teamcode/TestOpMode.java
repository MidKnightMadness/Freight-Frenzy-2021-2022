package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TestOpMode extends OpMode {
    SampleDrive drive;

    @Override
    public void init() {
        drive = new SampleDrive(hardwareMap);
    }

    @Override
    public void loop() {
        drive.setPos(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
