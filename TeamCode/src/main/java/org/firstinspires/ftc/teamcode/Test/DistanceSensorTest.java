package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
@Disabled
public class DistanceSensorTest extends OpMode {

    //sensor declaration
    private ModernRoboticsI2cRangeSensor rangeF;
    private ModernRoboticsI2cRangeSensor rangeL;
    private ModernRoboticsI2cRangeSensor rangeR;

    @Override
    public void init()
    {
        rangeF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORFRONT);
        telemetry.addData("rangeF", rangeF);
        rangeL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORLEFT);
        telemetry.addData("rangeL", rangeL);
        rangeR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORRIGHT);
        telemetry.addData("rangeR", rangeR);
    }

    @Override
    public void loop()
    {
        telemetry.addData("DistF", rangeF.getDistance(DistanceUnit.INCH));
        telemetry.addData("DistL", rangeL.getDistance(DistanceUnit.INCH));
        telemetry.addData("DistR", rangeR.getDistance(DistanceUnit.INCH));
    }
}