package org.firstinspires.ftc.teamcode.Test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Outtake.SampleOuttake;

import java.util.concurrent.Callable;

@Autonomous
public class ShootTest extends LinearOpMode{

    private Outtake out = new SampleOuttake();

    @Override
    public void runOpMode()
    {
        out.init(hardwareMap, telemetry);

        waitForStart();

        out.start();
        sleep(2000);
        out.resetFeed();
        sleep(1000);
        out.feed();
        sleep(2500);
        out.resetFeed();
        sleep(1000);
        out.feed();
        sleep(2000);
        out.resetFeed();
        sleep(1000);
        out.feed();
        sleep(2000);
        out.resetFeed();
        sleep(1000);
        out.stop();
    }



}
