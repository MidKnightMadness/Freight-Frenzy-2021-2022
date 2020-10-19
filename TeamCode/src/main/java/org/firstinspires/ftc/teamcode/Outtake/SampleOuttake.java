package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SampleOuttake extends Outtake{

    //declare intake motor
    private DcMotor motor;

    //initialize motor`
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get("Intake");
    }

    //start motor
    @Override
    public void start() {
        motor.setPower(1);
    }

    //stop motor
    @Override
    public void stop() {
        motor.setPower(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) {
        motor.setPower(speed);
    }
}