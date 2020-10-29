package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Intake.SampleIntake;

@TeleOp
public class Toggle extends OpMode {

    private int toggle = 0;
    private boolean lastLeftBumper = false;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if(!lastLeftBumper && gamepad1.left_bumper) {
            toggle = 1;
        }
        else if(lastLeftBumper && gamepad1.left_bumper) {
            toggle = 0;
        }

        if(toggle == 1) {
            telemetry.addLine("enabled");
        }
        if(toggle == 0) {
            telemetry.addLine("disabled");
        }

        lastLeftBumper = gamepad1.left_bumper;
    }
}
