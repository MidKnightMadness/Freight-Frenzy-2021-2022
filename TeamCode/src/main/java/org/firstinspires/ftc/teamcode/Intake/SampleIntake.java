package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Config;

public class SampleIntake extends Intake{

    //declare intake motor
    private DcMotor intakeMotorL;
    private DcMotor intakeMotorR;
    private Servo releaseServo;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        super.init(hardwareMap, telemetry, gamepad1, gamepad2);

        intakeMotorL = hardwareMap.dcMotor.get(Config.INTAKEL);
        intakeMotorR = hardwareMap.dcMotor.get(Config.INTAKER);
        releaseServo = hardwareMap.servo.get(Config.INTAKERELEASE);
    }

    @Override
    public void release() {
        releaseServo.setPosition(0);
    }

    //start motor
    @Override
    public void start() {
        intakeMotorL.setPower(0.95);
        intakeMotorR.setPower(-0.95);
    }

    //stop motor
    @Override
    public void stop() {
        intakeMotorL.setPower(0);
        intakeMotorR.setPower(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) {
        intakeMotorL.setPower(speed);
        intakeMotorR.setPower(-speed);
    }
}
