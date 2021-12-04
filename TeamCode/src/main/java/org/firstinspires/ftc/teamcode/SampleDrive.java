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

    public void drive(double x, double y, double rotation) {
        //FRMotor.setPower(x + y + rotation);
        //FLMotor.setPower(x - y + rotation);
        //BRMotor.setPower(-x + y + rotation);
        //BLMotor.setPower(-x - y + rotation);

        FRMotor.setVelocity((x - y + rotation) * 1000);
        FLMotor.setVelocity((x + y + rotation) * 1000);
        BRMotor.setVelocity((-x - y + rotation) * 1000);
        BLMotor.setVelocity((-x + y + rotation) * 1000);
    }

    public void setPos(double x, double y, double rotation) {
        /*FRMotor.setTargetPosition(0);//(int)((x - y + rotation) * 1000) + FRMotor.getTargetPosition());
        FLMotor.setTargetPosition(0);//(int)((x + y + rotation) * 1000) + FLMotor.getTargetPosition());
        BRMotor.setTargetPosition(0);//(int)((-x - y + rotation) * 1000) + BRMotor.getTargetPosition());
        BLMotor.setTargetPosition(0);//(int)((-x + y + rotation) * 1000) + BLMotor.getTargetPosition());*/
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setPower(1.0);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLMotor.setPower(1.0);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setPower(1.0);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setPower(1.0);
    }
}
