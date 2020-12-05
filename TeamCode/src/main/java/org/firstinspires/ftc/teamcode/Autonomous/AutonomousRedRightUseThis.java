package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

import java.io.File;
import java.io.PrintWriter;

@Autonomous
@Disabled
public class AutonomousRedRightUseThis extends LinearOpMode{

    private Drive drive = new SampleDrive();
    private Intake intake = new SampleIntake();
    private Outtake outtake = new SampleOuttake();
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    private SampleVisual visual = new SampleVisual();

    @Override
    public void runOpMode()
    {
        drive.init(hardwareMap, telemetry, gamepad1, gamepad2);
        intake.init(hardwareMap, telemetry, gamepad1, gamepad2);
        outtake.init(hardwareMap, telemetry, gamepad1, gamepad2);
        wobbleGoal.init(hardwareMap, telemetry, gamepad1, gamepad2);
        visual.init(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();

        wobbleGoal.close();
        sleep(1000);
        wobbleGoal.slightLift();

        drive.moveToPosition(-13,12);
        sleep(10);
        visual.update();
        drive.moveToPosition(0,12);

        if(visual.getStartStack() == Visual.STARTERSTACK.A)
        {
            drive.moveToPosition(-5,70);
            wobbleGoal.lower();
            wobbleGoal.open();
            drive.moveToPosition(-15,70);
        }
        else if (visual.getStartStack() == Visual.STARTERSTACK.B)
        {
            drive.moveToPosition(-15,90);
            wobbleGoal.lower();
            wobbleGoal.open();
            drive.moveToPosition(-25,90);
        }
        else
        {
            drive.moveToPosition(-5, 112);
            wobbleGoal.lower();
            wobbleGoal.open();
            drive.moveToPosition(-15,112);
        }


//        outtake.start();
        drive.moveToPower3();              //insert aiming position 3
//        outtake.feed();
        drive.moveToPower2();              //insert aiming position 2
//        outtake.feed();
        drive.moveToPower1();            //insert aiming position 1
//        outtake.feed();
//        outtake.stop();

        drive.move(0, 12);

        visual.stop();
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
