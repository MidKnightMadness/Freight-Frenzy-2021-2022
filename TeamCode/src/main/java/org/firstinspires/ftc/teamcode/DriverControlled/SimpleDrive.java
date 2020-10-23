package org.firstinspires.ftc.teamcode.DriverControlled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class SimpleDrive extends OpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    @Override
    public void init() {
        FL = hardwareMap.dcMotor.get(Config.DRIVEFL);
        FR = hardwareMap.dcMotor.get(Config.DRIVEFR);
        BL = hardwareMap.dcMotor.get(Config.DRIVEBL);
        BR = hardwareMap.dcMotor.get(Config.DRIVEBR);
    }

    @Override
    public void loop() {
        FL.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        FR.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        BL.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        BR.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
    }
}
