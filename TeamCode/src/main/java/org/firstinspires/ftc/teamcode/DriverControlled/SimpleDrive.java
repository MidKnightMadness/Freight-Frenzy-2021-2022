package org.firstinspires.ftc.teamcode.DriverControlled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
@Disabled
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

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        FL.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        FR.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        BL.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        BR.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);

//        telemetry.addData("FL vel", FL.getVelocity());
//        telemetry.addData("FR vel", FR.getVelocity());
//        telemetry.addData("BL vel", BL.getVelocity());
//        telemetry.addData("BR vel", BR.getVelocity());
        telemetry.addData("FL pos", FL.getCurrentPosition());
        telemetry.addData("FR pos", FR.getCurrentPosition());
        telemetry.addData("BL pos", BL.getCurrentPosition());
        telemetry.addData("BR pos", BR.getCurrentPosition());
    }
}
