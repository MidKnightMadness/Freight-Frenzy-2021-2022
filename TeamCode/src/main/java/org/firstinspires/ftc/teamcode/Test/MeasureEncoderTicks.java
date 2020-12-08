package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
@Disabled
public class MeasureEncoderTicks extends OpMode {
    DcMotor motorWobble;
    DcMotor motorOuttake;
    DcMotor motorIntakeL;
    DcMotor motorIntakeR;

    @Override
    public void init() {
        motorWobble = hardwareMap.dcMotor.get(Config.WOBBLEMOTOR);
        motorOuttake = hardwareMap.dcMotor.get(Config.OUTTAKEMOTOR);
        motorIntakeL = hardwareMap.dcMotor.get(Config.INTAKEL);
        motorIntakeR = hardwareMap.dcMotor.get(Config.INTAKER);
    }

    @Override
    public void loop() {
        telemetry.addData("wobble pos", motorWobble.getCurrentPosition());
        telemetry.addData("outtake pos", motorOuttake.getCurrentPosition());
        telemetry.addData("inttakeL pos", motorIntakeL.getCurrentPosition());
        telemetry.addData("inttakeR pos", motorIntakeR.getCurrentPosition());
    }
}
