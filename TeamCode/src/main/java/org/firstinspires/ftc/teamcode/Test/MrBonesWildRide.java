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

        drive.moveToPosition(1,1);
        telemetry.addLine("finished moving to (1,1)");
        telemetry.update();
        sleep(5000);
        drive.moveToPosition(0,0);
        telemetry.addLine("finished moving to (0,0)");
        telemetry.update();
        while(!isStopRequested()) {
        }
        telemetry.addLine("bingas");


    }
}