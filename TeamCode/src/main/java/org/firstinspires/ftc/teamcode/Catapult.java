package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Catapult {
    private DcMotor catapultMotor; //outtake
    private Servo headServo; //catapult flap
    private Servo flapServo; //catapult flap
    private int startPosition;

    public Catapult(HardwareMap hardwareMap) {
        catapultMotor = hardwareMap.get(DcMotor.class, "catapult");
        headServo = hardwareMap.get(Servo.class, "head");
        flapServo = hardwareMap.get(Servo.class, "flap");
        startPosition = catapultMotor.getCurrentPosition();
    }

    public void upper() {
        catapultMotor.setTargetPosition(500 + startPosition);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(0.1);
    }

    public void middle() {
        catapultMotor.setTargetPosition(400 + startPosition);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(0.1);
    }

    public void lower() {
        catapultMotor.setTargetPosition(300 + startPosition);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(0.1);
    }

    public void returnPosition() {
        catapultMotor.setTargetPosition(startPosition);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(0.1);
    }

    public void setPosition(double y, Telemetry telemetry) {
        catapultMotor.setTargetPosition(catapultMotor.getCurrentPosition() + (int)(-y * 100));
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(1);
        telemetry.addData("Catapult Motor Position", catapultMotor.getCurrentPosition());
        telemetry.update();
    }

    public void headLeft() {
        headServo.setPosition(0);
    }

    public void headReturn() {
        headServo.setPosition(1);
    }

    public void flapOn() {
        flapServo.setPosition(1);
    }

    public void flapOff() {
        flapServo.setPosition(0);
    }
}
