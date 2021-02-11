package org.firstinspires.ftc.teamcode.WobbleGoal;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    private final double open = 0.80;
    //close value
    private final double closed = 0.10;
    //lifted value (encoder ticks)
    private final int liftedPos = 350;

    //initialize motor
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        super.init(hardwareMap, telemetry, gamepad1, gamepad2);
        motor = hardwareMap.dcMotor.get(Config.WOBBLEMOTOR);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo1 = hardwareMap.servo.get(Config.WOBBLESERVO);
    }

    //move elevator up
    @Override
    public void lift() {
        motor.setTargetPosition(liftedPos);
        motor.setPower(1);
        telemetry.addLine("lifting wobble goal");
        telemetry.addData("wobble motor position", motor.getCurrentPosition());
    }

    //move elevator down
    @Override
    public void lower(){
        motor.setTargetPosition(0);
        motor.setPower(0);
        telemetry.addLine("lowering wobble goal");
    }

    //slightly lift the elevator
    @Override
    public void slightLift()
    {
        motor.setTargetPosition(100);
        motor.setPower(0.5);
        telemetry.addLine("slight lifting wobble goal");
    }

    @Override
    public void outputTelemetry()
    {
        telemetry.addLine("Current Wobble Position: " + motor.getCurrentPosition());
        telemetry.addLine("Target Wobble Position: " + motor.getTargetPosition());
    }

    //stop elevator motor
    @Override
    public void stop() {
        motor.setPower(0);
        telemetry.addLine("stopping wobble goal motor");
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