package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class WestCoastDrive {
    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;

    public WestCoastDrive(HardwareMap hardwareMap) {
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
    }

    public void drive(double y, double rotation) {
        FR.setPower(-y + rotation);
        FL.setPower(y + rotation);
        BR.setPower(y + rotation);
        BL.setPower(-y + rotation);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPos(double y, double rotation) {
        FR.setTargetPosition((int)(-y + rotation) + FR.getCurrentPosition());
        FL.setTargetPosition((int)(y + rotation) + FL.getCurrentPosition());
        BR.setTargetPosition((int)(-y + rotation) + BR.getCurrentPosition());
        BL.setTargetPosition((int)(y + rotation) + BL.getCurrentPosition());
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setPower(0.2);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(0.2);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setPower(0.2);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setPower(0.2);
        while(!atTarget()) { }
    }

    public boolean atTarget() {
        if(FR.getCurrentPosition() > FR.getTargetPosition() - 10 && FR.getCurrentPosition() < FR.getTargetPosition() + 10 &&
                FL.getCurrentPosition() > FL.getTargetPosition() - 10 && FL.getCurrentPosition() < FL.getTargetPosition() + 10 &&
                BR.getCurrentPosition() > BR.getTargetPosition() - 10 && BR.getCurrentPosition() < BR.getTargetPosition() + 10 &&
                BL.getCurrentPosition() > BL.getTargetPosition() - 10 && BL.getCurrentPosition() < BL.getTargetPosition() + 10)
            return true;
        else
            return false;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", FR.getCurrentPosition());
        telemetry.addData("FL Motor Position", FL.getCurrentPosition());
        telemetry.addData("BR Motor Position", BR.getCurrentPosition());
        telemetry.addData("BL Motor Position", BL.getCurrentPosition());
        telemetry.update();
    }
}
