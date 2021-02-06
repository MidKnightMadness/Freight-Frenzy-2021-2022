package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Chassis.Drive;
import org.firstinspires.ftc.teamcode.Chassis.SampleDrive;

import java.util.concurrent.Callable;

@Autonomous
public class MrBonesWildRide extends LinearOpMode {
    BNO055IMU imu;

    private Drive drive = new SampleDrive();

    @Override
    public void runOpMode() {
        drive.isStopRequested = new Callable<Boolean>() {@Override public Boolean call() {return isStopRequested();}};

        drive.init(hardwareMap, telemetry, gamepad1, gamepad2);
        waitForStart();

        drive.moveV2(0,12,0.4);
    }
}