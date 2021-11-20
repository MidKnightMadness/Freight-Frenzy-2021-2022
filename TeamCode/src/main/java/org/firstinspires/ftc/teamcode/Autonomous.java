package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    SampleDrive drive;
    SensorREV2MDistance distance_sensor;

    //Three Front 2 meter distance sensors
    private DistanceSensor sensorRangeL; //left sensor
    private DistanceSensor sensorRangeM; //middle sensor
    private DistanceSensor sensorRangeR; //right sensor

    @Override
    public void runOpMode() throws InterruptedException {
        int barcodeLocation = 0;
        drive = new SampleDrive(hardwareMap);
        sensorRangeL = hardwareMap.get(DistanceSensor.class, "sensor_range_left");
        sensorRangeM = hardwareMap.get(DistanceSensor.class, "sensor_range_middle");
        sensorRangeR = hardwareMap.get(DistanceSensor.class, "sensor_range_right");

        waitForStart();

        //while(opModeIsActive()) {
            // Detecting Team Shipping Element and Placing Pre-Load Box (7 seconds)
            telemetry.addData("deviceName", sensorRangeL.getDeviceName());
            telemetry.addData("Left Sensor Range", String.format("%.01f cm", sensorRangeL.getDistance(DistanceUnit.CM)));
            telemetry.addData("deviceName", sensorRangeM.getDeviceName());
            telemetry.addData("Middle Sensor Range", String.format("%.01f cm", sensorRangeM.getDistance(DistanceUnit.CM)));
            telemetry.addData("deviceName", sensorRangeR.getDeviceName());
            telemetry.addData("Right Sensor Range", String.format("%.01f cm", sensorRangeR.getDistance(DistanceUnit.CM)));
            telemetry.update();

            if (sensorRangeL.getDistance(DistanceUnit.CM) < sensorRangeM.getDistance(DistanceUnit.CM) && sensorRangeL.getDistance(DistanceUnit.CM) < sensorRangeR.getDistance(DistanceUnit.CM)) {
                barcodeLocation = 1;
            } else if (sensorRangeM.getDistance(DistanceUnit.CM) < sensorRangeR.getDistance(DistanceUnit.CM) && sensorRangeM.getDistance(DistanceUnit.CM) < sensorRangeL.getDistance(DistanceUnit.CM)) {
                barcodeLocation = 2;
            } else if (sensorRangeR.getDistance(DistanceUnit.CM) < sensorRangeM.getDistance(DistanceUnit.CM) && sensorRangeR.getDistance(DistanceUnit.CM) < sensorRangeL.getDistance(DistanceUnit.CM)){
                barcodeLocation = 3;
            } else {
                barcodeLocation = 0;
            }

            telemetry.addData("Barcode Location", barcodeLocation);
        //}
        drive.drive(-1, 0.8, 0); //drive to alliance shipping hub
        sleep(1500);

        drive.drive(0,0,0);
        sleep(1000);

        // Deliver Duck Through Carousel (5 seconds)
        //drive.drive(0,0,-1); //drive to carousel from shipping hub
        //sleep(1000);
        drive.drive(-1,-0.5,0);
        sleep(2500);
        drive.drive(0,0,0);
        sleep(1000);

         //Placing Duck on Alliance Shipping Hub
        drive.drive(1,0.5,0); //drive to alliance shipping hub from carousel
        sleep(2500);
        drive.drive(0,0,0);
        sleep(1000);

        // Placing 2 Freight from Warehouse to Alliance Shipping Hub (10 seconds)
        drive.drive(1, -0.8, 0); //drive to warehouse from alliance shipping hub
        sleep(1500);
        drive.drive(0,1,0);
        sleep(1000);

        drive.drive(0,0,0);
        sleep(1000);

        drive.drive(0,-1, 0); //drive to alliance shipping hub from warehouse
        sleep(1000);
        drive.drive(-1, 0.8, 0);
        sleep(1500);
        drive.drive(1,1,0);
        sleep(1000);

        // Completely Parking in Warehouse (4 seconds)
        // drive to warehouse from alliance shipping hub

        drive.drive(0,0,0);
    }

}
