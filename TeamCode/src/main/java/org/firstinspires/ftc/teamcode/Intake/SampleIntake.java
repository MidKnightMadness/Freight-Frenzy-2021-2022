package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Config;

public class SampleIntake extends Intake{

    //declare intake motor
    private DcMotor motor;
    private DcMotor motor2;

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get(Config.INTAKE);
    }

    //start motor
    @Override
    public void start() {
        motor.setPower(1);
        motor.set Power(-1);
    }

    //stop motor
    @Override
    public void stop() {
        motor.setPower(0);
        motor2.setPower(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) {
        motor.setPower(speed);
        motor2.setPower(-speed);
    }
}
