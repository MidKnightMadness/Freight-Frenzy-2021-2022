package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Chassis.Drive;
import org.firstinspires.ftc.teamcode.Chassis.SampleDrive;

@Autonomous
public class MrBonesWildRide extends LinearOpMode {
    private Drive drive = new SampleDrive();

    @Override
    public void runOpMode() {
        drive.init(hardwareMap, telemetry);
        waitForStart();

        drive.moveToPosition(10,10);
        //sleep(5000);
        drive.turn(180);
        //sleep(5000);
        drive.moveToPosition(0,0);
        while(!isStopRequested()) {
        }
        telemetry.addLine("bingas");


    }
}