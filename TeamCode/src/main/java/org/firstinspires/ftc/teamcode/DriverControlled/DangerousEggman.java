package org.firstinspires.ftc.teamcode.DriverControlled;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Chassis.Drive;
import org.firstinspires.ftc.teamcode.Chassis.SampleDrive;
import org.firstinspires.ftc.teamcode.Common.Config;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Intake.SampleIntake;
import org.firstinspires.ftc.teamcode.LEDs.LEDModes;
import org.firstinspires.ftc.teamcode.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Outtake.SampleOuttake;
import org.firstinspires.ftc.teamcode.WobbleGoal.SampleWobbleGoal;
import org.firstinspires.ftc.teamcode.WobbleGoal.WobbleGoal;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.LEDs.LED;
import org.firstinspires.ftc.teamcode.LEDs.LEDColor;

import static org.firstinspires.ftc.teamcode.LEDs.LED.Colors.BLUE;
import static org.firstinspires.ftc.teamcode.LEDs.LED.Colors.GREEN;
import static org.firstinspires.ftc.teamcode.LEDs.LED.Colors.OFF;
import static org.firstinspires.ftc.teamcode.LEDs.LED.Colors.RED;

import java.io.File;
import java.util.Scanner;

@TeleOp
public class DangerousEggman extends OpMode {

    private Drive drive = new SampleDrive();
    private Intake intake = new SampleIntake();
    private Outtake outtake = new SampleOuttake();
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    ModernRoboticsI2cRangeSensor sensorL;
    ModernRoboticsI2cRangeSensor sensorR;
    ModernRoboticsI2cRangeSensor sensorF;
    private int intToggle, outToggle, openWobToggle = 1, liftWobToggle = 0;
    private boolean lastLeftBumper, lastLeftBumper2, lastLeftTrigger, lastLeftTrigger2, lastRightBumper, lastBButton, lastXButton, lastYButton = false, slowMode;
    private double lastTime, distOffX, distOffY, turn;
    private double driveAngleOffset;



    /*
    reconfigured as according to drive team request

    gamepad1: intake and driving
    LBumper - toggle intake
    LTrigger - toggle intake reverse

    x - reset drive heading

    LStick - directional drive
    RStickX - turn

    A - deploy intake
    X - toggle slow drive
    RBumper - toggle slow drive



    gamepad2: wobble goal, outtake, and driver assist
    RBumper - toggle outtake flywheel for tower
    LBumper - toggle outtake flywheel for power shot
    LTrigger - toggle outtake flywheel for tower from corner
    RTrigger - toggle outtake feeder

    B - toggle wobble claw
    Y - toggle lift wobble goal

    NOTE: removed cancel driver assist since it is not allowed to block loop anyways

    A - drive to shoot at tower
    X - prevent driver assist A from moving backwards

    Dpad_up - middle power shot
    Dpad_down - top tower goal from current position
    Dpad_right - right power shot
    Dpad_left - left power shot
    */

    @Override
    public void init() {
        drive.init(hardwareMap, telemetry, gamepad1, gamepad2);
        intake.init(hardwareMap, telemetry, gamepad1, gamepad2);
        outtake.init(hardwareMap, telemetry, gamepad1, gamepad2);
        wobbleGoal.init(hardwareMap, telemetry, gamepad1, gamepad2);
        sensorL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORLEFT);
        sensorR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORRIGHT);
        sensorF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORFRONT);

        try {
            File fileName = new File("Coordinates.txt");
            Scanner inFile = new Scanner(fileName);
            drive.setCurrentX(inFile.nextDouble());
            drive.setCurrentY(inFile.nextDouble());
            drive.setAngle(inFile.nextDouble());
        }catch (Exception e){
            telemetry.addLine(e.getMessage());
        }

        //LED experiments below

        telemetry.addLine("Creating LEDs");
        try {
            I2cDeviceSynch leds = hardwareMap.get(I2cDeviceSynch.class, "ledstrip");
            LED.init(leds);
            LED.update();
        } catch (Exception e) {
            //e.printStackTrace(Log.out);
            //Log.out.close();
        }
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        LEDModes.telemetry = telemetry;
        LED.ALL.set(LED.Modes.STATIC,
                LED.Colors.OFF);
        LED.update();

    }

    @Override
    public void loop() {
        telemetry.addData("loop time", time - lastTime);
        lastTime = time;

        if(!gamepad2.a && !gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.dpad_left)
        {
            //drive controlled by left stick used to move forwards and sideways, right stick used to turn (gamepad 1)
            if(slowMode) {
                drive.drive(gamepad1.left_stick_y / 3, gamepad1.left_stick_x / 3, gamepad1.right_stick_x / 3);
                telemetry.addLine("slow mode enabled");
            } else {
                drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }
        //reset drive angle
        if(gamepad1.x)
        {
            driveAngleOffset = -drive.getAngle();
        }

        //x toggle button to enable slow mode
//        if(!lastXButton && (gamepad1.x || gamepad1.right_bumper)) {
//            slowMode = !slowMode;
//        }
//        lastXButton = (gamepad1.x || gamepad1.right_bumper);
        slowMode = gamepad1.right_bumper;

        //a button to deploy intake (gamepad 1)
        if(gamepad1.a){
            intake.deploy();
        }

        //when left bumper is pressed, start intake unless it was previously on in which intake stops (gamepad 1)
        //when left trigger is pressed, reverse intake unless it was previously on in which intake stops (gamepad 1)
        if(!lastLeftBumper && gamepad1.left_bumper) {
            if(intToggle == 1) {
                intToggle = 0;

            }
            else if(intToggle == 0 || intToggle == 2) {
                intToggle = 1;
            }
        }
        if(!lastLeftTrigger &&  gamepad1.left_trigger == 1) {
            if(intToggle == 2) {
                intToggle = 0;
            }
            else if(intToggle == 0 || intToggle == 1) {
                intToggle = 2;
            }
        }
        if(intToggle == 0) {
            intake.stop();
            LED.ALL.set(LED.Modes.STATIC,
                    LED.Colors.PINK);
            LED.update();
        }
        if(intToggle == 1) {
            intake.start();
            LED.ALL.set(LED.Modes.STATIC,
                    LED.Colors.GREEN);
            LED.update();
        }
        if(intToggle == 2) {
            intake.eject();
            LED.ALL.set(LED.Modes.STATIC,
                    LED.Colors.RED);
            LED.update();
        }
        lastLeftBumper = gamepad1.left_bumper;
        lastLeftTrigger = gamepad1.left_trigger == 1;

        //when right bumper is pressed, start outtake unless it was previously on in which outtake stops (gamepad 1)
        if(!lastRightBumper && gamepad2.right_bumper) {
            if(outToggle == 1) {
                outToggle = 0;
            }
            else {
                outToggle = 1;
            }
        }
        lastRightBumper = gamepad2.right_bumper;

        //when right bumper is pressed, start outtake unless it was previously on in which outtake stops (gamepad 2 power shot power)
        if(!lastLeftBumper2 && gamepad2.left_bumper) {
            if(outToggle == 2) {
                outToggle = 0;
            }
            else {
                outToggle = 2;
            }
        }
        lastLeftBumper2 = gamepad2.left_bumper;
        //when right trigger is pressed, start outtake unless it was previously on in which outtake stops (gamepad 2 power shot power)
        if(!lastLeftTrigger2 && gamepad2.left_trigger != 0) {
            if(outToggle == 3) {
                outToggle = 0;
            }
            else {
                outToggle = 3;
            }
        }
        lastLeftTrigger2 = gamepad2.left_trigger != 0;
        if(outToggle == 0) {
            outtake.stop();
        }
        else if(outToggle == 1) {
            outtake.start();
        }
        else if(outToggle == 2) {
            outtake.startPowerShot();
        }
        else if(outToggle == 3) {
            outtake.startFar();
        }

        //when right trigger is pressed, outtake servo is moved to push ring forwards unless (gamepad 1)
        //it is already in that position in which it moves back to its starting position (gamepad 1)
        if(gamepad2.right_trigger != 1) {
            outtake.resetFeed();
            telemetry.addLine("resetting feeder");
        }
        else {
            outtake.feedRun();
            telemetry.addLine("feeding");
        }

        //when b button is pressed, wobble goal is opened unless it is already opened in which it closes (gamepad 1)
        if(!lastBButton && gamepad2.b) {
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
        lastBButton = gamepad2.b;

        //when y button is pressed, wobble goal is lifted unless it is already lifted in which it lowers (gamepad 1)
        if(!lastYButton && gamepad2.y) {
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
//        wobbleGoal.outputTelemetry();
        lastYButton = gamepad2.y;

        //when a button is pressed, start adjusting bot to shooting position in front of tower unless it already is in which it stops (gamepad 2)
        if(gamepad2.a) {
            //adjust using distance sensors
            distOffX = (sensorR.getDistance(DistanceUnit.INCH) - 28) / 36;
            distOffY = sensorF.getDistance(DistanceUnit.INCH);
            if(distOffY != 0)
                distOffY = -(distOffY - 65) / 36;
//            turn = (drive.getAngle() + driveAngleOffset) / 30;
            turn = (drive.getAngle() + driveAngleOffset);
            turn = Math.sqrt(Math.abs(turn)) * Math.signum(turn) / 10;

            //discard unusual output
            if(Math.abs(distOffX) > 1000)
                distOffX = 0;
            if(Math.abs(distOffY) > 1000)
                distOffY = 0;
            //manual override backwards movement
            if(distOffY > 0 && gamepad2.x)
                distOffY = 0;

            drive.drive(distOffY, distOffX, turn);
        }
        telemetry.addData("distX", distOffX);
        telemetry.addData("distY", distOffY);
        telemetry.addData("turn", turn);

        /*
        //when x button is pressed, start adjusting bot to shooting position in front of leftmost power shot unless it already is in which it stops (gamepad 2)
        if(gamepad2.dpad_left) {
            //adjust using distance sensors
            distOffX = (sensorR.getDistance(DistanceUnit.INCH) - 27) / 48;
            distOffY = -(sensorF.getDistance(DistanceUnit.INCH) - 62) / 48;
            turn = (drive.getAngle() + driveAngleOffset) / 30;

            //discard unusual output
            if(Math.abs(distOffX) > 1000)
                distOffX = 0;
            if(Math.abs(distOffY) > 1000)
                distOffY = 0;

            drive.drive(distOffY, distOffX, turn);
        }

        //when button is pressed, start adjusting bot to shooting position in front of center power shot unless it already is in which it stops (gamepad 2)
        if(gamepad2.dpad_up) {
            //adjust using distance sensors
            distOffX = (sensorR.getDistance(DistanceUnit.INCH) - 27) / 48;
            distOffY = -(sensorF.getDistance(DistanceUnit.INCH) - 62) / 48;
            turn = (drive.getAngle() + driveAngleOffset) / 30;

            //discard unusual output
            if(Math.abs(distOffX) > 1000)
                distOffX = 0;
            if(Math.abs(distOffY) > 1000)
                distOffY = 0;

            drive.drive(distOffY, distOffX, turn);
        }


        //when b button is pressed, start adjusting bot to shooting position in front of rightmost power shot unless it already is in which it stops (gamepad 2)
        if(gamepad2.dpad_right) {
            //adjust using distance sensors
            distOffX = (sensorR.getDistance(DistanceUnit.INCH) - 27) / 48;
            distOffY = -(sensorF.getDistance(DistanceUnit.INCH) - 62) / 48;
            turn = (drive.getAngle() + driveAngleOffset) / 30;

            //discard unusual output
            if(Math.abs(distOffX) > 1000)
                distOffX = 0;
            if(Math.abs(distOffY) > 1000)
                distOffY = 0;

            drive.drive(distOffY, distOffX, turn);
        }
        */


        if(gamepad2.dpad_down)
        {
            //start outtake according to velocity
            outtake.startFromPos(drive.getCurrentX(), drive.getCurrentY(), 5);

            //turn to tower
            double angle = (drive.getAngle() + driveAngleOffset);
            double targetAngle = Math.toDegrees(Math.atan2(-28.75 - drive.getCurrentX(), 80 - drive.getCurrentY()));  //get the angle that we want to turn to
            drive.drive(0,0, angle);
        }

        if(outToggle == 1) {
            telemetry.addLine("Outtake: Tower Speed");
        }
        else if(outToggle == 2) {
            telemetry.addLine("Outtake: Power Shot Speed");
        }
        else if(outToggle == 3) {
            telemetry.addLine("Outtake: Far");
        }
        else{
            telemetry.addLine("Outtake: OFF");
        }
        telemetry.addData("Outtake Velocity", outtake.getVelocity());
        telemetry.addData("Target Velocity", outtake.getTargetVelocity());

        telemetry.addData("Current X", drive.getCurrentX());
        telemetry.addData("Current Y", drive.getCurrentY());
        telemetry.addData("Current Angle", (drive.getAngle() + driveAngleOffset));
        telemetry.update();

    }

    @Override
    public void stop() {
        LED.ALL.set(LED.Modes.STATIC, OFF);
        LED.update();
        telemetry.addLine("Stopping");
    }
}
