package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    SampleDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleDrive(hardwareMap);

        waitForStart();

        // Detecting Team Shipping Element and Placing Pre-Load Box (7 seconds)
        drive.drive(-1, 0.8, 0); //drive to alliance shipping hub
        sleep(1000);

        drive.drive(0,0,0);
        sleep(3000);

        // Deliver Duck Through Carousel (5 seconds)
        //drive.drive(0,0,-1); //drive to carousel from shipping hub
        //sleep(1000);
        drive.drive(-1,-0.5,0);
        sleep(2000);

        drive.drive(0, 0, 0);
        sleep(3000);

        // Placing Duck on Alliance Shipping Hub
        /*drive.drive(0,0,-1); //drive to alliance shipping hub from carousel
        sleep(2000);
        drive.drive(0,1,0);
        sleep(2000);

        // Placing 2 Freight from Warehouse to Alliance Shipping Hub (10 seconds)
        drive.drive(0,0,1); //drive to warehouse from alliance shipping hub
        sleep(500);
        drive.drive(0,1,0);
        sleep(1000);

        drive.drive(0,0,-1); //drive to alliance shipping hub from warehouse
        sleep(200);
        drive.drive(0,1,0);
        sleep(1000);*/

        // Completely Parking in Warehouse (4 seconds)
        //drive to warehouse from alliance shipping hub

        drive.drive(0,0,0);

    }
}
