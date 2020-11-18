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

@Autonomous
public class AutoRedRightManual extends LinearOpMode {
    private Drive drive = new SampleDrive();
    private Intake intake = new SampleIntake();
    private Outtake outtake = new SampleOuttake();
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    private SampleVisual visual = new SampleVisual();

    @Override
    public void runOpMode()
    {
        drive.init(hardwareMap, telemetry);
        //intake.init(hardwareMap, telemetry);
        //outtake.init(hardwareMap, telemetry);
        //wobbleGoal.init(hardwareMap, telemetry);
        visual.init(hardwareMap, telemetry);

        waitForStart();

        //wobbleGoal.close();
        //Do we lift it????

        visual.update();
        if(visual.getStartStack() == Visual.STARTERSTACK.A)
        {
            drive.move(-3,70);
        }
        else if (visual.getStartStack() == Visual.STARTERSTACK.B)
        {
            drive.move(-23, 94);
        }
        else
        {
            drive.move(-3,128);
        }

        //wobbleGoal.open();

        if(visual.getStartStack() == Visual.STARTERSTACK.A)
        {
            drive.move(-26.25,-8);
        }
        else if (visual.getStartStack() == Visual.STARTERSTACK.B)
        {
            drive.move(-26.25, -32);
        }
        else
        {
            drive.move(-26.25,-66);
        }

        //outtake.stop();
        //outtake.feed();
        drive.move(-7.5,0);
        //outtake.feed();
        drive.move(-7.5,0);
        //outtake.feed();
        //outtake.stop();
    }
}
