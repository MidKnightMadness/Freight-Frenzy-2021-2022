package org.firstinspires.ftc.teamcode.WobbleGoal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Common.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Config;

public class SampleWobbleGoal extends WobbleGoal{

    //declare motor
    private DcMotor motor;
    //declare servo
    private Servo servo1;

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get(Config.WOBBLEM);
        servo1 = hardwareMap.servo.get(Config.WOBBLES);
    }

    //start elevator motor
    @Override
    public void start() {
        motor.setPower(1);
    }

    //stop elevator motor
    @Override
    public void stop() {
        motor.setPower(0);
    }

    //manually set elevator motor speed
    @Override
    public void setSpeed(double speed) {
        motor.setPower(speed);
    }

    //Set arm position
    @Override
    public void setPos(double pos)
    {
        servo1.setPosition(pos);
    }
}