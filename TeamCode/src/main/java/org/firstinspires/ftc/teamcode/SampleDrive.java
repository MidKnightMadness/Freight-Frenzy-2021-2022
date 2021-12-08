package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        FRMotor.setTargetPosition((int)(( x - y + rotation) * 200) + FRMotor.getCurrentPosition());
        FLMotor.setTargetPosition((int)(( x + y + rotation) * 200) + FLMotor.getCurrentPosition());
        BRMotor.setTargetPosition((int)((-x - y + rotation) * 200) + BRMotor.getCurrentPosition());
        BLMotor.setTargetPosition((int)((-x + y + rotation) * 200) + BLMotor.getCurrentPosition());
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setPower(1.0);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLMotor.setPower(1.0);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setPower(1.0);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setPower(1.0);
        while(!atTarget()) { }
    }

    public boolean atTarget() {
        if(FRMotor.getCurrentPosition() > FRMotor.getTargetPosition() - 1 && FRMotor.getCurrentPosition() < FRMotor.getTargetPosition() + 1 &&
                FLMotor.getCurrentPosition() > FLMotor.getTargetPosition() - 1 && FLMotor.getCurrentPosition() < FLMotor.getTargetPosition() + 1 &&
                BRMotor.getCurrentPosition() > BRMotor.getTargetPosition() - 1 && BRMotor.getCurrentPosition() < BRMotor.getTargetPosition() + 1 &&
                BLMotor.getCurrentPosition() > BLMotor.getTargetPosition() - 1 && BLMotor.getCurrentPosition() < BLMotor.getTargetPosition() + 1)
            return true;
        else
            return false;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("FR Motor Position", FRMotor.getCurrentPosition());
        telemetry.addData("FL Motor Position", FLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Position", BRMotor.getCurrentPosition());
        telemetry.addData("BL Motor Position", BLMotor.getCurrentPosition());
        telemetry.update();
    }
}
