package org.firstinspires.ftc.teamcode.Visual;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class DistanceSensorTest extends OpMode {

    //sensor declaration
    private Rev2mDistanceSensor distF;
    private Rev2mDistanceSensor distL;
    private Rev2mDistanceSensor distR;

    @Override
    public void init()
    {
        distF = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORFRONT);
        distL = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORLEFT);
        distR = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORRIGHT);
    }

    @Override
    public void loop()
    {
        telemetry.addData("DistF", distF.getDistance(DistanceUnit.INCH));
        telemetry.addData("DistL", distL.getDistance(DistanceUnit.INCH));
        telemetry.addData("DistR", distR.getDistance(DistanceUnit.INCH));
    }
}