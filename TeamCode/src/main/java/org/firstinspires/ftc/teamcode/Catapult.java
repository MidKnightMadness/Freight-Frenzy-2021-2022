package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Catapult {
    private DcMotor catapultMotor; //outtake

    public Catapult(HardwareMap hardwareMap) {
        catapultMotor = hardwareMap.get(DcMotor.class, "catapult");
    }

    public void upper() {
        catapultMotor.setTargetPosition(300);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(1.0);
    }

    public void middle() {
        catapultMotor.setTargetPosition(400);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(1.0);
    }

    public void lower() {
        catapultMotor.setTargetPosition(500);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(1.0);
    }

    public void returnPosition() {
        catapultMotor.setTargetPosition(0);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(1.0);
    }
}
