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
        double distOffx = (distR.getDistance(DistanceUnit.INCH) - 24) / 10;
        double distOffy = (distF.getDistance(DistanceUnit.INCH) - 24) / 10;
        double turn = gamepad1.right_stick_x;
        telemetry.addData("distance to right", distOffx);
        telemetry.addData("distance to front", distOffy);

        if(!gamepad1.a || distOffy >= 200){
            distOffy = -gamepad1.left_stick_y;
        }
        if(!gamepad1.b || distOffx >= 200){
            distOffx = gamepad1.left_stick_x;
        }
        drive.drive(distOffy, -distOffx, turn);
    }
}
