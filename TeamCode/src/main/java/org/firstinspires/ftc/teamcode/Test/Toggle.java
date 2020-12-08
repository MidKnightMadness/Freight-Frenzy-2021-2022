package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Intake.SampleIntake;

@TeleOp
@Disabled
public class Toggle extends OpMode {

    private int toggle = 0;
    private boolean lastLeftBumper = false;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if(!lastLeftBumper && gamepad1.left_bumper) {
            if(toggle == 1) {
                toggle = 0;
            }
            else if(toggle == 0) {
                toggle = 1;
            }
        }

        if(toggle == 0) {
            telemetry.addLine("disabled");
        }
        if(toggle == 1) {
            telemetry.addLine("enabled");
        }

        lastLeftBumper = gamepad1.left_bumper;
    }
}
