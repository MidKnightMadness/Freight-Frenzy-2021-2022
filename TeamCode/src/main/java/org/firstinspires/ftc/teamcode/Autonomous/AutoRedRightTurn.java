package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Chassis.Drive;
import org.firstinspires.ftc.teamcode.Chassis.SampleDrive;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Intake.SampleIntake;
import org.firstinspires.ftc.teamcode.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Outtake.SampleOuttake;
import org.firstinspires.ftc.teamcode.Visual.SampleVisual;
import org.firstinspires.ftc.teamcode.Visual.Visual;
import org.firstinspires.ftc.teamcode.WobbleGoal.SampleWobbleGoal;
import org.firstinspires.ftc.teamcode.WobbleGoal.WobbleGoal;

import java.util.concurrent.Callable;

import java.io.PrintWriter;
import java.io.File;

@Autonomous


public class AutoRedRightTurn extends LinearOpMode {
    private Drive drive = new SampleDrive();
    private Intake intake = new SampleIntake();
    private Outtake outtake = new SampleOuttake();
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    private SampleVisual visual = new SampleVisual();

    @Override
    public void runOpMode() {
        //create stop requested callable
        Callable<Boolean> stopRequestedCall = new Callable<Boolean>() {@Override public Boolean call() {return isStopRequested();}};
        drive.init(hardwareMap, telemetry, gamepad1, gamepad2);
        drive.isStopRequested = stopRequestedCall;
        telemetry.addLine("Drive initialized!");
        telemetry.update();
        intake.init(hardwareMap, telemetry, gamepad1, gamepad2);
        outtake.init(hardwareMap, telemetry, gamepad1, gamepad2);
        wobbleGoal.init(hardwareMap, telemetry, gamepad1, gamepad2);
        visual.init(hardwareMap, telemetry, gamepad1, gamepad2);
        telemetry.addLine("Visual initialized!");
        telemetry.update();

        while(!isStopRequested() && !isStarted())
            idle();

        if(isStopRequested())
        {
            return;
        }

        outtake.startPowerShot();

        wobbleGoal.close();
        sleep(1000);
        wobbleGoal.slightLift();

        //move up to starting stack
        drive.move(-13, 12);
        sleep(10); //Reason?
        //get starting stack
        visual.update();
        telemetry.addLine("Zone: " + visual.getStartStack());
        telemetry.update();
        drive.move(13, 0);


        //move to correct drop zone
        if (visual.getStartStack() == Visual.STARTERSTACK .A) {
            drive.move(0, 58);
            telemetry.addLine("Zone: " + visual.getStartStack());
            telemetry.update();
        } else if (visual.getStartStack() == Visual.STARTERSTACK.B) {
            drive.move(-40, 80);
            telemetry.addLine("Zone: " + visual.getStartStack());
            telemetry.update();
        } else {
            drive.move(0, 105);
            telemetry.addLine("Zone: " + visual.getStartStack());
            telemetry.update();
        }

        //release wobble goal
        wobbleGoal.lower();
        wobbleGoal.open();
        drive.move(-10, 0);

        //move to shooting position 3
        if (visual.getStartStack() == Visual.STARTERSTACK.A) {
            drive.move(-32.25, -21);
            telemetry.addLine("Zone: " + visual.getStartStack());
            telemetry.update();
        } else if (visual.getStartStack() == Visual.STARTERSTACK.B) {
            drive.move(-7.25, -43);
            telemetry.addLine("Zone: " + visual.getStartStack());
            telemetry.update();
        } else {
            drive.move(-32.25, -68);
            telemetry.addLine("Zone: " + visual.getStartStack());
            telemetry.update();
        }

        drive.adjustWalls(60, 46);

        telemetry.addLine("waiting for outtake");
        telemetry.update();

        //shoot power shots
        for(int i = 0; i < 10; i++)  //make sure outtake is really ready
            while(!outtake.isReady())
            {
                idle();
                telemetry.update();
            }

        telemetry.addLine("Outtake ready, starting to shoot");
        telemetry.update();
        outtake.feedRun();
        sleep(1000);
        outtake.resetFeed();
        sleep(1000);

        drive.turnToPoint(-31.5,144);
        outtake.feedRun();
        sleep(1000);
        outtake.resetFeed();
        sleep(1000);

        drive.turnToPoint(-39,144);
        outtake.feedRun();
        sleep(1000);
        outtake.resetFeed();
        sleep(1000);

        outtake.stop();
        drive.move(0, 14);
        drive.adjustWalls(46,53.5);

        visual.stop();
        telemetry.addLine("Program End :)");
        telemetry.update();

        try {
            PrintWriter outFile = new PrintWriter(new File("Coordinates.txt"));
            outFile.println(drive.getCurrentX());
            outFile.println(drive.getCurrentY());
            outFile.println(drive.getAngle());
            outFile.close();
        }catch (Exception e){
            telemetry.addLine(e.getMessage());
        }
    }
}