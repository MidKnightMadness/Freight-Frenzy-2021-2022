package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class MeasureEncoderTicks extends OpMode {
    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get(Config.WOBBLEMOTOR);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
        telemetry.addData("power", motor.getPower());
        telemetry.addData("position", motor.getCurrentPosition());
    }
}
