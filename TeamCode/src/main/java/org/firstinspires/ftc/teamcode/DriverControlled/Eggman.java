package org.firstinspires.ftc.teamcode.DriverControlled;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Chassis.Drive;
import org.firstinspires.ftc.teamcode.Chassis.SampleDrive;
import org.firstinspires.ftc.teamcode.Common.Config;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Intake.SampleIntake;
import org.firstinspires.ftc.teamcode.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Outtake.SampleOuttake;
import org.firstinspires.ftc.teamcode.WobbleGoal.WobbleGoal;
import org.firstinspires.ftc.teamcode.WobbleGoal.SampleWobbleGoal;

import java.util.Scanner;
import java.io.File;
import java.util.concurrent.Callable;

@TeleOp
public class Eggman extends OpMode {

    private Drive drive = new SampleDrive();
    private Intake intake = new SampleIntake();
    private Outtake outtake = new SampleOuttake();
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    Rev2mDistanceSensor distL;
    Rev2mDistanceSensor distR;
    Rev2mDistanceSensor distF;
    private int intToggle, outToggle, outFeedToggle, openWobToggle, liftWobToggle, lowerWobToggle, towerAdjustToggle = 0;
    private boolean lastLeftBumper, lastRightBumper, lastRightTrigger, lastBButton, lastYButton, lastAButton, lastAButton2 = false;

    @Override
    public void init() {
        drive.init(hardwareMap, telemetry, gamepad1, gamepad2);
        intake.init(hardwareMap, telemetry, gamepad1, gamepad2);
        outtake.init(hardwareMap, telemetry, gamepad1, gamepad2);
        wobbleGoal.init(hardwareMap, telemetry, gamepad1, gamepad2);
        distL = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORLEFT);
        distR = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORRIGHT);
        distF = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORFRONT);

        try {
            File fileName = new File("Coordinates.txt");
            Scanner inFile = new Scanner(fileName);
            drive.setCurrentX(inFile.nextDouble());
            drive.setCurrentY(inFile.nextDouble());
            drive.setAngle(inFile.nextDouble());
        }catch (Exception e){
            telemetry.addLine(e.getMessage());
        }
    }

    @Override
    public void loop() {
        //drive controlled by left stick and right stick
        drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if(gamepad1.x) {
            drive.moveToTower();
        }

        //intake is a toggle controlled by left bumper
        if(!lastLeftBumper && gamepad1.left_bumper) {
            if(intToggle == 1) {
                intToggle = 0;
            }
            else if(intToggle == 0) {
                intToggle = 1;
            }
        }
        if(intToggle == 0) {
            intake.stop();
        }
        if(intToggle == 1) {
            intake.start();
        }
        lastLeftBumper = gamepad1.left_bumper;

        //outtake is a toggle controlled by right bumper
        if(!lastRightBumper && gamepad1.right_bumper) {
            if(outToggle == 1) {
                outToggle = 0;
            }
            else if(outToggle == 0) {
                outToggle = 1;
            }
        }
        if(outToggle == 0) {
            outtake.stop();
        }
        if(outToggle == 1) {
            outtake.start();
        }
        lastRightBumper = gamepad1.right_bumper;

        //outtake servo is a toggle controlled by right trigger
        if(!lastRightTrigger && gamepad1.right_trigger == 1) {
            if(outFeedToggle == 1) {
                outFeedToggle = 0;
            }
            else if(outFeedToggle == 0) {
                outFeedToggle = 1;
            }
        }
        if(outFeedToggle == 0) {
            outtake.resetFeed();
        }
        if(outFeedToggle == 1) {
            outtake.feedRun();
        }
        lastRightTrigger = gamepad1.right_trigger == 1;

        //wobble goal toggles between open and close using b button
        if(!lastBButton && gamepad1.b) {
            if(openWobToggle == 1) {
                openWobToggle = 0;
            }
            else if(openWobToggle == 0) {
                openWobToggle = 1;
            }
        }
        if(openWobToggle == 0) {
            wobbleGoal.close();
        }
        if(openWobToggle == 1) {
            wobbleGoal.open();
        }
        lastBButton = gamepad1.b;

        //lifting with y button
        if(!lastYButton && gamepad1.y) {
            if(liftWobToggle == 1) {
                liftWobToggle = 0;
            }
            else if(liftWobToggle == 0) {
                liftWobToggle = 1;
            }
        }
        if(liftWobToggle == 0) {
            wobbleGoal.lower();
        }
        if(liftWobToggle == 1) {
            wobbleGoal.lift();
        }
        lastYButton = gamepad1.y;

        //lower with a button
        if(!lastAButton && gamepad1.a) {
            if (lowerWobToggle == 1) {
                lowerWobToggle = 0;
            }
        }
        if(!lastAButton2 && gamepad2.a) {
            if(towerAdjustToggle == 1) {
                towerAdjustToggle = 0;
            }
            else if(towerAdjustToggle == 0) {
                towerAdjustToggle = 1;
            }
        }
        if(towerAdjustToggle == 1) {
            //adjust using distance sensors
            double distOffX = (distR.getDistance(DistanceUnit.INCH) - 17.5);
            double distOffY = -(distF.getDistance(DistanceUnit.INCH) - 65);
            double turn = drive.getAngle();

            if(distOffX < 1) {
                distOffX = 0;
            }
            if(distOffY > 1) {
                distOffY = 0;
            }
            if(turn > 5 || turn < -5) {
                turn = 0;
            }
        }
        lastAButton2 = gamepad2.a;


        telemetry.addData("Current X", drive.getCurrentX());
        telemetry.addData("Current Y", drive.getCurrentY());
        telemetry.addData("Current Angle", drive.getAngle());
        telemetry.update();
    }

}
