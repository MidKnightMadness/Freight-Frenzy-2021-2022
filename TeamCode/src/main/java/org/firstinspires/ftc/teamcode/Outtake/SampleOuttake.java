package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Outtake.Outtake;


public class SampleOuttake extends Outtake {

    //declare intake motor
    DcMotor motor;
    //DcMotor motor2; in case of double-motor outtake

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get("Outtake");
        //motor2 = hardwareMap.dcMotor.get("Intake2")
    }

    //start motor
    @Override
    public void start() {
        motor.setPower(1);
        //motor2.setPower(1);
    }

    //stop motor
    @Override
    public void stop() {
        motor.setPower(0);
        //motor2.setPower(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) {
        motor.setPower(speed);
        //motor2.setPower(speed);
    }
}