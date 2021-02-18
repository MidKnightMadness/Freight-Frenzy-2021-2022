package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class testAaron extends OpMode {
    @Override
    public void init() {
        telemetry.addLine("Hello World!");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            telemetry.addLine("a");
        } else telemetry.addLine("");
        telemetry.update();
    }
}
