package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SampleIntake extends Intake{

    DcMotor motor;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get("Intake");
    }

    @Override
    public void start() {
        motor.setPower(1);
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void setSpeed(double speed) {
        motor.setPower(speed);
    }
}
