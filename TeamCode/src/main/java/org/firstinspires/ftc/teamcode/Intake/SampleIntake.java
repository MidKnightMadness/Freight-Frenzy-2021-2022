package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Config;

public class SampleIntake extends Intake{

    //declare intake motor
    private DcMotor motorL;
    private DcMotor motorR;

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motorL = hardwareMap.dcMotor.get(Config.INTAKEL);
        motorR = hardwareMap.dcMotor.get(Config.INTAKER)
    }

    //start motor
    @Override
    public void start() {
        motorL.setPower(1);
        motorR.setPower(-1);
    }

    //stop motor
    @Override
    public void stop() {
        motorL.setPower(0);
        motorR.setPower(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) {
        motorL.setPower(speed);
        motorR.setPower(-speed);
    }
}
