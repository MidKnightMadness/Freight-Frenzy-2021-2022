package org.firstinspires.ftc.teamcode.Autonomous.FinalAutonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Arm.Arm;
import org.firstinspires.ftc.teamcode.Carousel.Carousel;
import org.firstinspires.ftc.teamcode.Chassis.Drivechain;

@Autonomous
public class RedLeftSide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Drivechain AutonomousDC = new Drivechain(hardwareMap);
        Carousel AutonomousCS = new Carousel(hardwareMap);
        Arm arm = new Arm(telemetry, hardwareMap);
        AutonomousDC.calibrateGyro();
        AutonomousDC.resetGyro();
        AutonomousDC.resetTicks();
        waitForStart();


        /**************************Driving Controls****************************
         * ====================================================================
        //Forward
        AutonomousDC.moveRobot(-925, 925, -925, 925, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Counterclockwise 30 degrees
        AutonomousDC.moveRobot( 300, 300, 300, 300, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Clockwise 30 degrees
        AutonomousDC.moveRobot(-300, -300, -300, -300, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);

        //Forward
        AutonomousDC.moveRobot(-925, 925, -925, 925, 0.25, 0.25, 0.25, 0.25);
        sleep(3000);
         *======================================================================
         **********************************************************************/


        //Turn to carousel
        sleep(2500);
        AutonomousDC.moveRobot(-800, 800, -800, 800, 1, 1, 1, 1);
        AutonomousDC.turnDeg(-120.0f, telemetry);
        sleep(2000);
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        AutonomousDC.resetTicks();
        sleep(1000);

        //Carousel
        AutonomousCS.CarouselAutonomous(2000,-0.3);
        sleep(1000);
        AutonomousDC.turnDeg(120.0f, telemetry);
        sleep(1000);
        AutonomousDC.resetTicks();

        //Go to the alliance hub
        AutonomousDC.moveRobot(400, -400, 400, -400, 1, 1, -1, -1);
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        sleep(1000);
        AutonomousDC.resetTicks();
        AutonomousDC.turnDeg(90, telemetry);
        sleep(1000);
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(400, -400, 400, -400, 1, 1, -1, -1);
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        sleep(1000);

        //Outtake the x block
        arm.setWinchPosition(1800);
        arm.armSetPosition(0.5);
        sleep(1000);
        arm.cubeMotorRelease();
        sleep(1000);
        arm.cubeMotorReset();
        arm.resetArmPosition();
        arm.resetWinchPosition();

        //Parking
        AutonomousDC.turnDeg(-120.0f, telemetry);
        sleep(2000);
        AutonomousDC.resetTicks();
        AutonomousDC.moveRobot(950, -950, 950, -950, 1, 1, -1, -1);
        sleep(2000);
        while (AutonomousDC.isbusy()) {
            sleep(100);
        }
        AutonomousDC.resetTicks();

    }
}