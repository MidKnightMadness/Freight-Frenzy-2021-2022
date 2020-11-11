package org.firstinspires.ftc.teamcode.DriverControlled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class VelocityDrive extends OpMode {
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;

    @Override
    public void init() {
        FL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFL);
        FR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFR);
        BL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBL);
        BR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBR);

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
        FL.setVelocity((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 250);
        FR.setVelocity((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 250);
        BL.setVelocity((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 250);
        BR.setVelocity((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 250);

        telemetry.addData("FL vel", FL.getVelocity());
        telemetry.addData("FR vel", FR.getVelocity());
        telemetry.addData("BL vel", BL.getVelocity());
        telemetry.addData("BR vel", BR.getVelocity());
        telemetry.addData("FL pos", FL.getCurrentPosition());
        telemetry.addData("FR pos", FR.getCurrentPosition());
        telemetry.addData("BL pos", BL.getCurrentPosition());
        telemetry.addData("BR pos", BR.getCurrentPosition());
    }
}
