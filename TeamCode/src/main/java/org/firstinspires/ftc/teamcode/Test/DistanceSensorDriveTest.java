package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Chassis.Drive;
import org.firstinspires.ftc.teamcode.Chassis.SampleDrive;
import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class DistanceSensorDriveTest extends OpMode {

    Drive drive = new SampleDrive();
    Rev2mDistanceSensor distL;
    Rev2mDistanceSensor distR;
    Rev2mDistanceSensor distF;

    boolean alignMode;

    @Override
    public void init() {
        drive.init(hardwareMap, telemetry);
        distL = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORLEFT);
        distR = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORRIGHT);
        distF = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORFRONT);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            double dist = distL.getDistance(DistanceUnit.INCH) - 48;
            drive.drive( 0, dist, 0);
            telemetry.addData("distance", dist);
        }
        else{
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}
