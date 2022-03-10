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
    private Servo flapServo;

    public void init() {
        catapultMotor = hardwareMap.get(DcMotorEx.class, "catapult");
        flapServo = hardwareMap.get(Servo.class, "flap");
        catapultMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        startPosition = catapultMotor.getCurrentPosition();
        i = 0;
    }

    public void loop() {
        if(gamepad1.dpad_up)
            i++;
        if(gamepad1.dpad_down)
            i--;
        if (gamepad1.x) {
            flapServo.setPosition(0);
        }
        if (gamepad1.y) {
            flapServo.setPosition(0.5);
        }

        catapultMotor.setTargetPosition(startPosition  + i);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(1);
        telemetry.addData("Catapult Motor Current", catapultMotor.getCurrentPosition());
        telemetry.addData("Catapult Motor Target", catapultMotor.getTargetPosition());
        telemetry.addData("i", i);
        telemetry.update();
    }
}
