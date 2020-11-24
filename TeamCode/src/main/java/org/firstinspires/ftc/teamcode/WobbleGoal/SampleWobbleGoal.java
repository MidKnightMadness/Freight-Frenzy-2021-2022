package org.firstinspires.ftc.teamcode.WobbleGoal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Common.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ImmersiveMode;

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
    //lifted value (encoder ticks)
    private final int liftedPos = -2000;

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motor = hardwareMap.dcMotor.get(Config.WOBBLEMOTOR);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servo1 = hardwareMap.servo.get(Config.WOBBLESERVO);
    }

    //move elevator up
    @Override
    public void lift() {
        motor.setTargetPosition(liftedPos);
        motor.setPower(1);
    }

    //move elevator down
    @Override
    public void lower(){
        motor.setTargetPosition(0);
        motor.setPower(1);
    }

    //slightly lift the elevator
    @Override
    public void slightLift()
    {
        motor.setTargetPosition(-500);
        motor.setPower(0.5);
    }

    @Override
    public void outputTelemetry()
    {
        telemetry.addLine("Current Position: " + motor.getCurrentPosition());
        telemetry.addLine("Target Position: " + motor.getTargetPosition());
        telemetry.update();
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