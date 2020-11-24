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
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.left_stick_y);
        telemetry.addData("power", motor.getPower());
        telemetry.addData("position", motor.getCurrentPosition());
    }
}
