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
        drive.init(hardwareMap, telemetry, gamepad1, gamepad2);
        distL = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORLEFT);
        distR = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORRIGHT);
        distF = hardwareMap.get(Rev2mDistanceSensor.class, Config.DISTANCESENSORFRONT);
    }

    @Override
    public void loop() {
        double powX = (distR.getDistance(DistanceUnit.INCH) - 24) / 10;
        double powY = -(distF.getDistance(DistanceUnit.INCH) - 24) / 10;
        double turn = drive.getAngle() / 100;
        telemetry.addData("distance to right", powX);
        telemetry.addData("distance to front", powY);

        if(!gamepad1.a || powY >= 200){
            powY = gamepad1.left_stick_y;
        }
        if(!gamepad1.b || powX >= 200){
            powX = gamepad1.left_stick_x;
        }
        if(!gamepad1.x){
            turn = gamepad1.right_stick_x;
        }
        telemetry.addData("diving y", powY);
        telemetry.addData("diving x", -powX);
        telemetry.addData("turning", turn);
        drive.drive(powY, powX, turn);
    }
}
