package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    private DcMotor liftMotor;
    private Servo liftServo;
    private int motorStartPosition;

    public Lift(HardwareMap hardwareMap) {
        liftServo = hardwareMap.get(Servo.class, "ship_element_servo");
        liftMotor = hardwareMap.get(DcMotor.class, "ship_element_motor");
        motorStartPosition = liftMotor.getCurrentPosition();
    }

    public void lift() {
        liftMotor.setTargetPosition(300 + motorStartPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void lower() {
        liftMotor.setTargetPosition(motorStartPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void open() {
        liftServo.setPosition(100);
    }

    public void close() {
        liftServo.setPosition(0);
    }
}
