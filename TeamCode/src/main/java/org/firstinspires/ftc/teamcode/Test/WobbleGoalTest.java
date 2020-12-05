package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.WobbleGoal.WobbleGoal;
import org.firstinspires.ftc.teamcode.WobbleGoal.SampleWobbleGoal;

@TeleOp
@Disabled
public class WobbleGoalTest extends OpMode {
    private WobbleGoal wobbleGoal = new SampleWobbleGoal();
    private int openWobToggle, liftWobToggle  = 0;
    private boolean lastBButton, lastYButton  = false;

    @Override
    public void init() {
        wobbleGoal.init(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        //wobble goal toggles between open and close using a button
        if(!lastBButton && gamepad1.b) {
            if(openWobToggle == 1) {
                openWobToggle = 0;
            }
            else if(openWobToggle == 0) {
                openWobToggle = 1;
            }
        }
        if(openWobToggle == 0) {
            wobbleGoal.close();
        }
        if(openWobToggle == 1) {
            wobbleGoal.open();
        }
        lastBButton = gamepad1.b;

        //wobble goal toggles betweent lift and lower position
        if(!lastYButton && gamepad1.y) {
            if(liftWobToggle == 1) {
                liftWobToggle = 0;
            }
            else if(liftWobToggle == 0) {
                liftWobToggle = 1;
            }
        }
        if(liftWobToggle == 0) {
            wobbleGoal.lower();
        }
        if(liftWobToggle == 1) {
            wobbleGoal.lift();
        }
        lastYButton = gamepad1.y;
    }
}
