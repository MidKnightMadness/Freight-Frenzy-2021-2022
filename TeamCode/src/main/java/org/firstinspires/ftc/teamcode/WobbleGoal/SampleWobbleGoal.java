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
    //open value
    private final double open = 1.00;
    //close value
    private final double closed = 0.10;

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get(Config.WOBBLEMOTOR);
        servo1 = hardwareMap.servo.get(Config.WOBBLESERVO);
    }

    //move elevator up
    @Override
    public void lift() {
        motor.setPower(1);
    }

    //move elevator down
    @Override
    public void drop(){
        motor.setPower(-1);
    }

    //stop elevator motor
    @Override
    public void stop() {
        motor.setPower(0);
    }

    //Open claw
    @Override
    public void open() {
        servo1.setPosition(open);
    }

    //Close claw
    public void close() {
        servo1.setPosition(closed);
    }
}