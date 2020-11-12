package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class AutonomousRedRight extends LinearOpMode{

    private Drive drive = new SampleDrive();
    private Intake intake = new SampleIntake();
    private Outtake outtake = new SampleOuttake();
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    private SampleVisual visual = new SampleVisual();

    @Override
    public void runOpMode()
    {
        drive.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        outtake.init(hardwareMap, telemetry);
        wobbleGoal.init(hardwareMap, telemetry);
        visual.init(hardwareMap, telemetry);

        waitForStart();

        wobbleGoal.close();
        //Do we lift it????

        visual.update();
        if(visual.getStartStack() == Visual.STARTERSTACK.A)
        {
            /*
            insert zone A position here
            drive.moveToPosition();
             */
        }
        else if (visual.getStartStack() == Visual.STARTERSTACK.B)
        {
            /*
            insert position to side here
            drive.moveToPosition();
            insert zone B position here
            drive.moveToPosition();
             */
        }
        else
        {
            /*
            insert position to side here
            drive.moveToPosition();
            insert zone c position here
            drive.moveToPosition();
             */
        }

        wobbleGoal.open();

        //drive.moveToPosition();              insert aiming position 1
        outtake.start();
        outtake.feed();
        //drive.moveToPosition();              insert aiming position 2
        outtake.feed();
        //drive.moveToPosition();              insert aiming position 3
        outtake.feed();
        outtake.stop();
    }

}
