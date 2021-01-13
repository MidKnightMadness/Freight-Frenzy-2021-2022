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
import org.firstinspires.ftc.teamcode.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Outtake.SampleOuttake;
import org.firstinspires.ftc.teamcode.WobbleGoal.WobbleGoal;
import org.firstinspires.ftc.teamcode.WobbleGoal.SampleWobbleGoal;

import java.util.Scanner;
import java.io.File;

@TeleOp
public class Eggman extends OpMode {

    private Drive drive = new SampleDrive();
    private Intake intake = new SampleIntake();
    private Outtake outtake = new SampleOuttake();
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    ModernRoboticsI2cRangeSensor sensorL;
    ModernRoboticsI2cRangeSensor sensorR;
    ModernRoboticsI2cRangeSensor sensorF;
    private int intToggle, outToggle, outFeedToggle, openWobToggle = 1, liftWobToggle, towerAdjust, powerAdjust1, powerAdjust2, powerAdjust3 = 0;
    private boolean lastLeftBumper, lastLeftTrigger, lastRightBumper, lastRightTrigger, lastBButton, lastXButton, lastYButton, lastAButton2, lastXButton2, lastYButton2, lastBButton2 = false, slowMode;
    private double lastTime;

    /*
    gamepad1: manual
    LBumper - intake
    LTrigger - intake reverse
    RBumper - outtake start
    RTrigger - outtake shoot
    LStick - directional drive
    RStickX - turn
    A - deploy intake
    B - open wobble claw
    X - slow drive
    Y - lift wobble goal

    gamepad2: driver assist
    either trigger - cancel action
    move to:
    A - tower
    X - left power shot
    Y - middle power shot
    B - right power shot
    auto shoot:
    DPadUp - aim and shoot
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
    }

    @Override
    public void loop() {
        telemetry.addData("loop time", time - lastTime);
        lastTime = time;

        //drive controlled by left stick used to move forwards and sideways, right stick used to turn (gamepad 1)
        if(slowMode) {
            drive.drive(gamepad1.left_stick_y / 3, gamepad1.left_stick_x / 3, gamepad1.right_stick_x / 5);
            telemetry.addLine("slow mode enabled");
        } else {
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        //x toggle button to enable slow mode
        if(!lastXButton && gamepad1.x) {
            slowMode = !slowMode;
        }
        lastXButton = gamepad1.x;

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
        }
        if(intToggle == 1) {
            intake.start();
        }
        if(intToggle == 2) {
            intake.eject();
        }
        lastLeftBumper = gamepad1.left_bumper;
        lastLeftTrigger = gamepad1.left_trigger == 1;

        //when right bumper is pressed, start outtake unless it was previously on in which outtake stops (gamepad 1)
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

        //when right trigger is pressed, outtake servo is moved to push ring forwards unless (gamepad 1)
        //it is already in that position in which it moves back to its starting position (gamepad 1)
        if(!lastRightTrigger && gamepad1.right_trigger == 1) {
            if(outFeedToggle == 1) {
                outFeedToggle = 0;
            }
            else if(outFeedToggle == 0 && outtake.isReady()) {
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

        //when b button is pressed, wobble goal is opened unless it is already opened in which it closes (gamepad 1)
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

        //when y button is pressed, wobble goal is lifted unless it is already lifted in which it lowers (gamepad 1)
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
//        wobbleGoal.outputTelemetry();
        lastYButton = gamepad1.y;

        //when a button is pressed, start adjusting bot to shooting position in front of tower unless it already is in which it stops (gamepad 2)
        if(!lastAButton2 && gamepad2.a) {
            if(towerAdjust == 0) {
                towerAdjust = 1;
            }
        }
        if(towerAdjust == 1) {
            drive.alignForward();

            double distOffX = 1;
            double distOffY = 1;
            while(distOffX != 0 ||  distOffY != 0) {
                //adjust using distance sensors
                distOffX = (sensorR.getDistance(DistanceUnit.INCH) - 30.5);
                distOffY = -(sensorF.getDistance(DistanceUnit.INCH) - 63);

                if (distOffX < 1 && distOffX > -1) {
                    distOffX = 0;
                }
                if (distOffY < 1 && distOffY > -1) {
                    distOffY = 0;
                }

                drive.drive(distOffX / 10, distOffY / 10, 0);

                if(gamepad2.a) {
                    distOffX = 0;
                }
            }
            towerAdjust = 0;
        }
        lastAButton2 = gamepad2.a;

        //when x button is pressed, start adjusting bot to shooting position in front of leftmost power shot unless it already is in which it stops (gamepad 2)
        if(!lastXButton2 && gamepad2.x) {
            if(powerAdjust1 == 0) {
                powerAdjust1 = 1;
            }
        }
        if(powerAdjust1 == 1) {
            drive.alignForward();

            double distOffX = 1;
            double distOffY = 1;
            while(distOffX != 0 ||  distOffY != 0) {
                //adjust using distance sensors
                distOffX = (sensorL.getDistance(DistanceUnit.INCH) - 21.5);
                distOffY = -(sensorF.getDistance(DistanceUnit.INCH) - 63);

                if (distOffX < 1 && distOffX > -1) {
                    distOffX = 0;
                }
                if (distOffY < 1 && distOffY > -1) {
                    distOffY = 0;
                }

                drive.drive(distOffX / 10, distOffY / 10, 0);

                if(gamepad2.x) {
                    distOffX = 0;
                }
            }
            powerAdjust1 = 0;
        }
        lastAButton2 = gamepad2.a;

        //when y button is pressed, start adjusting bot to shooting position in front of center power shot unless it already is in which it stops (gamepad 2)
        if(!lastYButton2 && gamepad2.y) {
            if(powerAdjust2 == 0) {
                powerAdjust2 = 1;
            }
        }
        if(powerAdjust2 == 1) {
            drive.alignForward();

            double distOffX = 1;
            double distOffY = 1;
            while(distOffX != 0 ||  distOffY != 0) {
                //adjust using distance sensors
                distOffX = (sensorL.getDistance(DistanceUnit.INCH) - 29.5);
                distOffY = -(sensorF.getDistance(DistanceUnit.INCH) - 63);

                if (distOffX < 1 && distOffX > -1) {
                    distOffX = 0;
                }
                if (distOffY < 1 && distOffY > -1) {
                    distOffY = 0;
                }

                drive.drive(distOffX / 10, distOffY / 10, 0);

                if(gamepad2.y) {
                    distOffX = 0;
                }
            }
            towerAdjust = 0;
        }
        lastYButton2 = gamepad2.y;

        //when b button is pressed, start adjusting bot to shooting position in front of rightmost power shot unless it already is in which it stops (gamepad 2)
        if(!lastBButton2 && gamepad2.b) {
            if(powerAdjust3 == 0) {
                powerAdjust3 = 1;
            }
        }
        if(powerAdjust3 == 1) {
            drive.alignForward();

            double distOffX = 1;
            double distOffY = 1;
            while(distOffX != 0 ||  distOffY != 0) {
                //adjust using distance sensors
                distOffX = (sensorL.getDistance(DistanceUnit.INCH) - 36.5);
                distOffY = -(sensorF.getDistance(DistanceUnit.INCH) - 63);

                if (distOffX < 1 && distOffX > -1) {
                    distOffX = 0;
                }
                if (distOffY < 1 && distOffY > -1) {
                    distOffY = 0;
                }

                drive.drive(distOffX / 10, distOffY / 10, 0);

                if(gamepad2.b) {
                    distOffX = 0;
                }
            }
            powerAdjust3 = 0;
        }
        lastBButton2 = gamepad2.b;

        if(gamepad2.dpad_up)
        {
            //start up outtake
            outtake.startFromPos(drive.getCurrentX(), drive.getCurrentY(), 9);

            //turn to towergoal
            drive.turnToPoint(-28.75, 80);  //TODO: correct towergoal position –– (check tower position in SampleOuttake)
        }

        telemetry.addData("Current X", drive.getCurrentX());
        telemetry.addData("Current Y", drive.getCurrentY());
        telemetry.addData("Current Angle", drive.getAngle());
        telemetry.update();
    }

}
