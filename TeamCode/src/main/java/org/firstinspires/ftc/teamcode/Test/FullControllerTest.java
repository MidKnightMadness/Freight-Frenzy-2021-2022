package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class FullControllerTest extends OpMode {
    @Override
    public void init(){}

    @Override
    public void loop() {
        telemetry.addData("x pressed: "         , gamepad1.x);
        telemetry.addData("y pressed: "         , gamepad1.y);
        telemetry.addData("b pressed: "         , gamepad1.b);
        telemetry.addData("a pressed: "         , gamepad1.a);
        telemetry.addData("dpad down: "         , gamepad1.dpad_down);
        telemetry.addData("dpad right: "        , gamepad1.dpad_right);
        telemetry.addData("dpad up: "           , gamepad1.dpad_up);
        telemetry.addData("dpad left: "         , gamepad1.dpad_left);
        telemetry.addData("left stick x: "      , gamepad1.left_stick_x);
        telemetry.addData("left stick y: "      , gamepad1.left_stick_y);
        telemetry.addData("right stick x: "     , gamepad1.right_stick_x);
        telemetry.addData("right stick y: "     , gamepad1.right_stick_y);
        telemetry.addData("left stick button: " , gamepad1.left_stick_button);
        telemetry.addData("right stick button: ", gamepad1.right_stick_button);
        telemetry.addData("left trigger: "      , gamepad1.left_trigger);
        telemetry.addData("right trigger: "     , gamepad1.right_trigger);
        telemetry.addData("left bumper: "       , gamepad1.left_bumper);
        telemetry.addData("right bumper: "      , gamepad1.right_bumper);
        telemetry.addData("start pressed: "     , gamepad1.start);
        telemetry.addData("back pressed: "      , gamepad1.back);
        telemetry.update();
    }
}
