package org.firstinspires.ftc.teamcode.DriverControlled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class SimpleDrive extends OpMode {

    private DcMotor TL;
    private DcMotor TR;
    private DcMotor BL;
    private DcMotor BR;

    @Override
    public void init() {
        TL = hardwareMap.dcMotor.get(Config.DRIVEFL);
        TR = hardwareMap.dcMotor.get(Config.DRIVEFR);
        BL = hardwareMap.dcMotor.get(Config.DRIVEBL);
        BR = hardwareMap.dcMotor.get(Config.DRIVEBR);
    }

    @Override
    public void loop() {
        TL.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        TR.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        BL.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        BR.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
    }
}
