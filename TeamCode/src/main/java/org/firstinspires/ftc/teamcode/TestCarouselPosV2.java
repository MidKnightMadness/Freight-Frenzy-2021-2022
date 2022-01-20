package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp
public class TestCarouselPosV2 extends LinearOpMode {
    SampleDriveBigBird drive;

    private DistanceSensor sensorDistanceL; //left sensor
    private DistanceSensor sensorDistanceB; //back sensor
    private DistanceSensor sensorDistanceR; //right sensor
    BNO055IMU imu; //imu object
    Orientation angles;
    Acceleration gravity;
    double startTurning;

    @Override
    public void runOpMode() throws InterruptedException {
        //init
        drive = new SampleDriveBigBird(hardwareMap);
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorDistanceB = hardwareMap.get(DistanceSensor.class, "sensor_distance_back");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");
        //imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        waitForStart();
        while(opModeIsActive()){
            //telemetry
            telemetry.addData("deviceName", sensorDistanceL.getDeviceName());
            telemetry.addData("Left Sensor Range", String.format("%f in", sensorDistanceL.getDistance(DistanceUnit.INCH)));
            telemetry.addData("deviceName", sensorDistanceB.getDeviceName());
            telemetry.addData("Back Sensor Range", String.format("%f in", sensorDistanceB.getDistance(DistanceUnit.INCH)));
            telemetry.addData("deviceName", sensorDistanceR.getDeviceName());
            telemetry.addData("Right Sensor Range", String.format("%f in", sensorDistanceR.getDistance(DistanceUnit.INCH)));
            telemetry.addData("math of distance left: ", String.format("%f in", (sensorDistanceL.getDistance(DistanceUnit.INCH) - 9)/20));
            telemetry.addData("math of distance back: ", String.format("%f in", (sensorDistanceB.getDistance(DistanceUnit.INCH) - 10)/20));
            telemetry.addData("joystick value: ", gamepad1.left_stick_x);
            //drive to carousel spot un-angled
            if((sensorDistanceL.getDistance(DistanceUnit.INCH) >= 9) || (sensorDistanceB.getDistance(DistanceUnit.INCH) >= 10)) {
                drive.drive(-(sensorDistanceL.getDistance(DistanceUnit.INCH) - 9) / 20, -(sensorDistanceB.getDistance(DistanceUnit.INCH) - 10)/20, 0);
            } else {
                break;
            }

        }
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startTurning = angles.firstAngle;
        while(opModeIsActive()){
            if ((sensorDistanceB.getDistance(DistanceUnit.INCH) < 10) && (sensorDistanceL.getDistance(DistanceUnit.INCH) < 9)) {
                drive.drive(0, 0, 0);
                telemetry.addData("heading angle: ", formatAngle(angles.angleUnit, angles.firstAngle));
                telemetry.addData("startTurning: ", startTurning);
                telemetry.addLine("should be greater than");
                telemetry.addData("angles.firstAngle+37: ", angles.firstAngle+37);
                telemetry.update();
                if (startTurning > (angles.firstAngle + 37)) {
                    telemetry.addLine("trying to turn");
                    telemetry.update();
                    drive.drive(0, 0, (startTurning - 37) / 10);
                }
            }
        }
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
}
