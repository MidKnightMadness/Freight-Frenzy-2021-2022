package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class EncoderTickDrive extends OpMode {
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
        FL.setTargetPosition(0);
        FR.setTargetPosition(0);
        BL.setTargetPosition(0);
        BR.setTargetPosition(0);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);
    }

    @Override
    public void loop() {
//        FL.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
//        FR.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
//        BL.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
//        BR.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));

        if(gamepad1.a) {
            telemetry.addLine("a");
            FL.setTargetPosition(1000);
            FR.setTargetPosition(1000);
            BL.setTargetPosition(1000);
            BR.setTargetPosition(1000);
        }
        else if(gamepad1.b) {
            telemetry.addLine("b");
            FL.setTargetPosition(-1000);
            FR.setTargetPosition(-1000);
            BL.setTargetPosition(-1000);
            BR.setTargetPosition(-1000);
        }

//        telemetry.addData("FL vel", FL.getVelocity());
//        telemetry.addData("FR vel", FR.getVelocity());
//        telemetry.addData("BL vel", BL.getVelocity());
//        telemetry.addData("BR vel", BR.getVelocity());
        telemetry.addData("FL pos", FL.getCurrentPosition());
        telemetry.addData("FR pos", FR.getCurrentPosition());
        telemetry.addData("BL pos", BL.getCurrentPosition());
        telemetry.addData("BR pos", BR.getCurrentPosition());
        telemetry.addData("FL tpos", FL.getTargetPosition());
        telemetry.addData("FR tpos", FR.getTargetPosition());
        telemetry.addData("BL tpos", BL.getTargetPosition());
        telemetry.addData("BR tpos", BR.getTargetPosition());
    }
}
