package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class AutonomousMecanumRed1 extends LinearOpMode {
    SampleDrive drive;
    SensorREV2MDistance distance_sensor;

    //Three Front 2 meter distance sensors
    private DistanceSensor sensorRangeL; //left front sensor
    private DistanceSensor sensorRangeR; //right front sensor
    BNO055IMU imu;
    double offsetY;
    int[] barcodeLocationArray = new int[3]; //barcode location left, middle, and right
    int barcodeLocation = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        drive = new SampleDrive(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        sensorRangeL = hardwareMap.get(DistanceSensor.class, "sensor_range_left");
        sensorRangeR = hardwareMap.get(DistanceSensor.class, "sensor_range_right");
        sleep(1000);

        waitForStart();
        offsetY = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        /*while(opModeIsActive()) {
            drive.telemetry(telemetry);
        }*/

        // Detecting Team Shipping Element(2 seconds)
        if (sensorRangeL.getDistance(DistanceUnit.INCH) < 100 && sensorRangeL.getDistance(DistanceUnit.INCH) < sensorRangeR.getDistance(DistanceUnit.INCH)) {
            barcodeLocation = 0;
        } else if (sensorRangeR.getDistance(DistanceUnit.INCH) < 100 && sensorRangeR.getDistance(DistanceUnit.INCH) < sensorRangeL.getDistance(DistanceUnit.INCH)) {
            barcodeLocation = 1;
        } else {
            barcodeLocation = 2;
        }
        telemetry.addData("deviceName", sensorRangeL.getDeviceName());
        telemetry.addData("Left Sensor Range", String.format("%.01f in", sensorRangeL.getDistance(DistanceUnit.INCH)));
        telemetry.addData("deviceName", sensorRangeR.getDeviceName());
        telemetry.addData("Right Sensor Range", String.format("%.01f in", sensorRangeR.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Left Barcode Array", barcodeLocationArray[0]);
        telemetry.addData("Middle Barcode Array", barcodeLocationArray[1]);
        telemetry.addData("Right Barcode Array", barcodeLocationArray[2]);
        telemetry.addData("Barcode Location", barcodeLocation);
        telemetry.update();

        // Placing Pre-Load Box (5 seconds)
        drive.setPos(1400, 700, 0); //drive to alliance shipping hub
        //outtake pre-load box onto alliance shipping hub
        drive.telemetry(telemetry);
        orient();

        // Deliver Duck Through Carousel (5 seconds)
        drive.setPos(-2400,-1250, 0); //drive to carousel from shipping hub
        //rotate carousel
        //intake duck
        orient();

        //Placing Duck on Alliance Shipping Hub
        drive.setPos(2200,900,0); //drive to alliance shipping hub from carousel
        //outtake duck onto alliance shipping hub
        orient();

        // Placing 2 Freight from Warehouse to Alliance Shipping Hub (10 seconds)
        //for(int i = 0; i < 2; i++) {
        drive.setPos(0, -1100, -1000); //drive to warehouse from alliance shipping hub
        //drive.setPos(0,-2000,0);
        //intake freight
        //drive.setPos(0,2000,0); //drive to alliance shipping hub from warehouse
        drive.setPos(0, 1100, 1000);
        //outtake freight onto alliance shipping hub
        //}

        // Completely Parking in Warehouse (4 seconds)
        // drive to warehouse from alliance shipping hub
        drive.setPos(0, -1000, -1000); //drive to warehouse from alliance shipping hub
        //drive.setPos(0,-2000,0);
    }

    public void orient() {
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY < -5 ||
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY > 5) {
            drive.setPos(0, 0, (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY) * 10);
            sleep(1);
        }

    }

}

