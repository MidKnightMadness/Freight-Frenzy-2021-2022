package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestOpMode extends OpMode {
    SampleDrive drive;

    private DistanceSensor sensorDistanceL; //left front sensor
    private ModernRoboticsI2cRangeSensor sensorRangeM; //middle front range sensor
    private DistanceSensor sensorDistanceR; //right front sensor

    @Override
    public void init() {
        drive = new SampleDrive(hardwareMap);
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");
    }

    @Override
    public void loop() {
        //drive to shipping hub position
        if((sensorRangeM.getDistance(DistanceUnit.INCH) <= 9 || sensorRangeM.getDistance(DistanceUnit.INCH) >= 11) && gamepad1.a) {
            drive.drive(0, (sensorRangeM.getDistance(DistanceUnit.INCH) - 10)/100, 0);
        } else {
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            drive.telemetry(telemetry);
        }
        telemetry.addData("deviceName", sensorDistanceL.getDeviceName());
        telemetry.addData("Left Sensor Range", String.format("%.01f in", sensorDistanceL.getDistance(DistanceUnit.INCH)));
        telemetry.addData("deviceName", sensorRangeM.getDeviceName());
        telemetry.addData("Left Sensor Range", String.format("%.01f in", sensorRangeM.getDistance(DistanceUnit.INCH)));
        telemetry.addData("deviceName", sensorDistanceR.getDeviceName());
        telemetry.addData("Left Sensor Range", String.format("%.01f in", sensorDistanceR.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }
}
