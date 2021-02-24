package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
//@Disabled
public class RangeSensorTest extends OpMode {
    ModernRoboticsI2cRangeSensor sensor;

    @Override
    public void init() {
        telemetry.addLine("getting sensor");
        sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        telemetry.addLine("got sensor");
        telemetry.addData("sensor", sensor);
    }

    @Override
    public void loop() {
        telemetry.addData("distance"      , sensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("raw ultrasonic", sensor.rawUltrasonic());
        telemetry.addData("raw optical"   , sensor.rawOptical());
        telemetry.addData("cm ultrasonic" , sensor.cmUltrasonic());
        telemetry.addData("cm optical"    , sensor.cmOptical());
    }
}
