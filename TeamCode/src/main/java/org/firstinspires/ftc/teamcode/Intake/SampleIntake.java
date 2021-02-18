package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Config;

public class SampleIntake extends Intake{

    //declare intake motor
    private DcMotorEx intakeMotorL;
    private DcMotorEx intakeMotorR;
    private Servo releaseServo;
    private CRServo roller;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        super.init(hardwareMap, telemetry, gamepad1, gamepad2);

        intakeMotorL = (DcMotorEx) hardwareMap.dcMotor.get(Config.INTAKEL);
        intakeMotorR = (DcMotorEx) hardwareMap.dcMotor.get(Config.INTAKER);
        roller = (CRServo) hardwareMap.crservo.get(Config.INTAKEROLLER);
        releaseServo = hardwareMap.servo.get(Config.INTAKERELEASE);
    }

    @Override
    public void deploy() {
        //set position to 0.35 to lock (storing number in case if it's needed)
        releaseServo.setPosition(1);
    }

    //start motor
    @Override
    public void start() {
        intakeMotorL.setVelocity(2250);
        intakeMotorR.setVelocity(-2250);
        roller.setPower(-1);

        telemetry.addData("intake L", intakeMotorL.getVelocity());
        telemetry.addData("intake R", intakeMotorR.getVelocity());
    }

    //reverse motors to eject stuck rings
    @Override
    public void eject() {
        intakeMotorL.setVelocity(-2500);
        intakeMotorR.setVelocity(2500);
        roller.setPower(1);
    }

    //stop motor
    @Override
    public void stop() {
        intakeMotorL.setPower(0);
        intakeMotorR.setPower(0);
        roller.setPower(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) {
        intakeMotorL.setPower(speed);
        intakeMotorR.setPower(-speed);
    }
}
