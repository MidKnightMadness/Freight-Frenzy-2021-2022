package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
/*
Player 1 (gamepad1)
left stick      (hold)      shifting
right stick     (hold)      rotating
*/

@TeleOp
public class WestCoastOpMode extends OpMode {
    WestCoastDrive drive;

    @Override
    public void init() {
        drive = new WestCoastDrive(hardwareMap);
    }

    @Override
    public void loop() {
        //telemetry
        drive.telemetry(telemetry);

        //DRIVER ASSIST
        drive.drive(gamepad1.left_stick_y, gamepad1.right_stick_x); //drive function
    }
}
