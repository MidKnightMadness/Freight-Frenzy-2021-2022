package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class GamepadTest extends OpMode {
    @Override
    public void init() {

    }

    /*
     * Testing if the gamepad will update asynchronously (independent of the code)
     * Basically, if the gamepad's values will update while in a loop
     *
     * This is especially important for driver assist (safety) features
     * If it does update asynchronously, we can use the gamepad for canceling driver assist
     *
     * yes, it updates asynchronously
     */
    @Override
    public void loop() {

        if(gamepad1.a)
        {
            while (!gamepad1.x)
            {
                telemetry.addData("gamepad1 left stick x", gamepad1.left_stick_x);
                telemetry.addData("gamepad1 left stick y", gamepad1.left_stick_y);
                telemetry.addLine("press x to end");
                telemetry.update();
            }
            telemetry.addLine("while loop exited");
            telemetry.update();
        }
    }
}
