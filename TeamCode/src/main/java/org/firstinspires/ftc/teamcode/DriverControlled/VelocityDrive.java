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
    }

    @Override
    public void loop() {
        FL.setVelocity((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 1000);
        FR.setVelocity((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 1000);
        BL.setVelocity((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 1000);
        BR.setVelocity((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 1000);

        telemetry.addData("FL", FL.getVelocity());
        telemetry.addData("FR", FR.getVelocity());
        telemetry.addData("BL", BL.getVelocity());
        telemetry.addData("BR", BR.getVelocity());
    }
}
