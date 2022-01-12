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
        if((sensorRangeM.getDistance(DistanceUnit.INCH) <= 4.5 || sensorRangeM.getDistance(DistanceUnit.INCH) >= 5.5) && gamepad1.a && sensorRangeM.getDistance(DistanceUnit.INCH) < 100) {
            drive.drive(0, (sensorRangeM.getDistance(DistanceUnit.INCH) - 5)/10, 0);
        } else {
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            drive.catapult(gamepad1.right_trigger);
            drive.surgicalTubing(gamepad1.right_bumper);
            drive.telemetry(telemetry);
        }
    }
}
