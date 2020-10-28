package org.firstinspires.ftc.teamcode.DriverControlled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Chassis.Drive;
import org.firstinspires.ftc.teamcode.Chassis.SampleDrive;
import org.firstinspires.ftc.teamcode.Common.Config;
import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Intake.SampleIntake;
import org.firstinspires.ftc.teamcode.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Outtake.SampleOuttake;
import org.firstinspires.ftc.teamcode.WobbleGoal.WobbleGoal;
import org.firstinspires.ftc.teamcode.WobbleGoal.SampleWobbleGoal;

@TeleOp
public class Eggman extends OpMode {

    private Drive drive = new SampleDrive();
    private Intake intake = new SampleIntake();
    private Outtake outtake = new SampleOuttake();
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    private int toggle = 0;

    @Override
    public void init() {
        drive.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        outtake.init(hardwareMap, telemetry);
        wobbleGoal.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if(gamepad1.right_bumper) {
            intake.start();
        }
        else {
            intake.stop();
        }

        if(gamepad1.left_bumper && toggle == 0) {
            toggle = 1;
        }
        else if (gamepad1.left_bumper && toggle == 1) {
            toggle = 0;
        }

        if(toggle == 1) {
            intake.start();
        }
        else if(toggle == 0) {
            intake.stop();
        }

        if(gamepad1.a) {
            wobbleGoal.open();
        }
        else {
            wobbleGoal.stop();
        }
        if(gamepad1.b) {
            wobbleGoal.close();
        }
        else {
            wobbleGoal.stop();
        }
    }

}
