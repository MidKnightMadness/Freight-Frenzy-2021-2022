package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TestMotor extends OpMode {
    private DcMotorEx catapultMotor;
    private int i;
    private int startPosition;

    public void init() {
        catapultMotor = hardwareMap.get(DcMotorEx.class, "catapult");
        catapultMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        startPosition = catapultMotor.getCurrentPosition();
        i = 1000;
    }

    public void loop() {
        if(gamepad1.dpad_up)
            i++;

        catapultMotor.setTargetPosition(startPosition  + i);
        telemetry.addData("Catapult Motor Current", catapultMotor.getCurrentPosition());
        telemetry.addData("Catapult Motor Target", catapultMotor.getTargetPosition());
        telemetry.addData("i", i);
        telemetry.update();
    }
}
