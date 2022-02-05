package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class AutonomousMecanumRed2 extends LinearOpMode {
    SampleDrive drive;
    Catapult catapult;
    Carousel carousel;
    Intake intake;
    Lift lift;
    SensorREV2MDistance distance_sensor;

    //Three Front 2 meter distance sensors
    private DistanceSensor sensorDistanceL; //left front sensor
    private ModernRoboticsI2cRangeSensor sensorRangeM; //middle front range sensor
    private DistanceSensor sensorDistanceR; //right front sensor
    BNO055IMU imu;
    double offsetY;
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
        catapult = new Catapult(hardwareMap);
        carousel = new Carousel(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");
        sleep(1000);

        waitForStart();
        offsetY = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Detecting Team Shipping Element(0 seconds)
        if (sensorDistanceL.getDistance(DistanceUnit.INCH) < 100 && sensorDistanceL.getDistance(DistanceUnit.INCH) < sensorDistanceR.getDistance(DistanceUnit.INCH)) {
            barcodeLocation = 0;
        } else if (sensorDistanceR.getDistance(DistanceUnit.INCH) < 100) {
            barcodeLocation = 1;
        } else {
            barcodeLocation = 2;
        }
        telemetry.addData("Left Sensor Range", String.format("%.01f in", sensorDistanceL.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Right Sensor Range", String.format("%.01f in", sensorDistanceR.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Barcode Location", barcodeLocation);
        telemetry.update();

        // Placing Pre-Load Box (5 seconds)
        drive.setPos(-1400, -700, 0, telemetry); //drive to alliance shipping hub
        //outtake pre-load box onto alliance shipping hub
        drive.telemetry(telemetry);
        intake.dropIntake();
        sleep(2000);
        //orient();

        double time = getRuntime();
        while(getRuntime() < 2 + time)
            catapult.upper();
        sleep(1000);
        catapult.headReturn();
        sleep(3000);
        catapult.flapOn();
        sleep(3000);
        catapult.returnPosition();

        //values must be fixed
        drive.setPos(0, -900, -1000, telemetry); //drive to warehouse from alliance shipping hub
        //intake.surgicalTubingOn(); //intake freight
        drive.setPos(0,-4000,0, telemetry);

        // Deliver Duck Through Carousel (5 seconds)
        //drive.setPos(2400,1250, 0, telemetry); //drive to carousel from shipping hub
        //rotate carousel
        //intake duck
        //orient();

        // Completely Parking in Warehouse (4 seconds)
        // drive to warehouse from alliance shipping hub
        //drive.setPos(1000, -1000, 0, telemetry); //drive to warehouse from alliance shipping hub
        //drive.setPos(0,-2000,0);
    }

    public void orient() {
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY < -5 ||
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY > 5) {
            drive.setPos(0, 0, (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - offsetY) * 10, telemetry);
            sleep(1);
        }

    }

}

