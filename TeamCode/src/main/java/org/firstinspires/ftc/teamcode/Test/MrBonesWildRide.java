package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Chassis.Drive;
import org.firstinspires.ftc.teamcode.Chassis.SampleDrive;

import java.util.concurrent.Callable;

@Autonomous
public class MrBonesWildRide extends LinearOpMode {
    private Drive drive = new SampleDrive();

    @Override
    public void runOpMode() {
        Callable<Boolean> stopRequestedCall = new Callable<Boolean>() {@Override public Boolean call() {return isStopRequested();}};
        drive.isStopRequested = stopRequestedCall;

        drive.init(hardwareMap, telemetry);
        waitForStart();

        drive.turn(20);
        sleep(1000);
        drive.alignForward();
        telemetry.update();
        while(!isStopRequested()) {
        }
        telemetry.addLine("bingas");


    }
}