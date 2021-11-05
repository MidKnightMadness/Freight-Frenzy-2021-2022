package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class SampleDrive {
    //DcMotor FRMotor;
    //DcMotor FLMotor;
    //DcMotor BRMotor;
    //DcMotor BLMotor;

    DcMotorEx FRMotor;
    DcMotorEx FLMotor;
    DcMotorEx BRMotor;
    DcMotorEx BLMotor;

    public SampleDrive(HardwareMap hardwareMap) {
        //FRMotor = hardwareMap.dcMotor.get("FR");
        //FLMotor = hardwareMap.dcMotor.get("FL");
        //BRMotor = hardwareMap.dcMotor.get("BR");
        //BLMotor = hardwareMap.dcMotor.get("BL");

        FRMotor = hardwareMap.get(DcMotorEx.class, "FR");
        FLMotor = hardwareMap.get(DcMotorEx.class, "FL");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BR");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BL");
    }

    public void drive(double xx, double yy, double rotation) {
        //FRMotor.setPower(xx + yy + rotation);
        //FLMotor.setPower(xx - yy + rotation);
        //BRMotor.setPower(-xx + yy + rotation);
        //BLMotor.setPower(-xx - yy + rotation);

        FRMotor.setVelocity((xx - yy + rotation) * 1000);
        FLMotor.setVelocity((xx + yy + rotation) * 1000);
        BRMotor.setVelocity((-xx - yy + rotation) * 1000);
        BLMotor.setVelocity((-xx + yy + rotation) * 1000);

    }
}
