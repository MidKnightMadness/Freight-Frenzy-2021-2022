package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Common.Config;

@Disabled
@TeleOp
public class GetPID extends LinearOpMode {
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFL);
        FR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFR);
        BL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBL);
        BR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBR);

        telemetry.addData("FL PID", FL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("FR PID", FR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("BL PID", BL.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("BR PID", BR.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.update();

        waitForStart();

        while(!isStopRequested()) ;
    }
}
