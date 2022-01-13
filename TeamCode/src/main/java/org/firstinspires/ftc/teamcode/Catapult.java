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
    DcMotorEx FRMotor;
    DcMotorEx FLMotor;
    DcMotorEx BRMotor;
    DcMotorEx BLMotor;

    private DcMotor catapultMotor; //outtake
    private DcMotor surgicalTubingMotor; //intake
    private CRServo flapServo; //intake flap

    private DistanceSensor sensorDistanceL; //left front sensor
    ModernRoboticsI2cRangeSensor sensorRangeM;
    private DistanceSensor sensorDistanceR; //right front sensor

    public Catapult(HardwareMap hardwareMap) {
        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");

        catapultMotor = hardwareMap.get(DcMotor.class, "Catapult");
        surgicalTubingMotor = hardwareMap.get(DcMotor.class, "Surgical Tubing");
        flapServo = hardwareMap.get(CRServo.class, "Intake Flap");

        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");
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
