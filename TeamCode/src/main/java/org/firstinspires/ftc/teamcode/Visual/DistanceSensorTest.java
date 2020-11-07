package org.firstinspires.ftc.teamcode.Visual;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends OpMode {

    //sensor declaration
    private Rev2mDistanceSensor distanceSensor;

    @Override
    public void init()
    {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
    }

    @Override
    public void loop()
    {
        telemetry.addData("Distance from wall (in.): ", distanceSensor.getDistance(DistanceUnit.INCH));
    }
}
