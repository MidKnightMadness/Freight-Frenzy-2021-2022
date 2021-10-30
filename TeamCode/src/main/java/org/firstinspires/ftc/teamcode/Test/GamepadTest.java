package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp
public class GamepadTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {

        telemetry.addData("gamepad2 left stick x", gamepad2.left_stick_x);
        telemetry.addData("gamepad2 left stick y", gamepad2.left_stick_y);
        telemetry.addData("gamepad2 left_trigger", gamepad2.left_trigger);
        telemetry.addData("gamepad2 right_trigger", gamepad2.right_trigger);
        telemetry.addLine("press x to end");
        telemetry.update();
    }
}
