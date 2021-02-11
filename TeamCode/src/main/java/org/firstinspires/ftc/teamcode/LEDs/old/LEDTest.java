package org.firstinspires.ftc.teamcode.LEDs.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.LEDs.LED;
import org.firstinspires.ftc.teamcode.LEDs.LEDColor;

import static org.firstinspires.ftc.teamcode.LEDs.LED.Colors.BLUE;
import static org.firstinspires.ftc.teamcode.LEDs.LED.Colors.GREEN;
import static org.firstinspires.ftc.teamcode.LEDs.LED.Colors.OFF;
import static org.firstinspires.ftc.teamcode.LEDs.LED.Colors.RED;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
//@Disabled
@TeleOp

public class LEDTest extends OpMode {
    /* Declare OpMode members. */


    @Override
    public void init() {
        //Log.init();
        //Log.out.println("Creating LEDs");
        telemetry.addLine("Creating LEDs");
        try {
            I2cDeviceSynch leds = hardwareMap.get(I2cDeviceSynch.class, "ledstrip");
            LED.init(leds);
            LED.update();
        } catch (Exception e) {

        }
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //LED.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        LED.ALL.set(LED.Modes.BOUNCING,
                LED.Colors.RED);
        LED.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        LED.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        LED.ALL.set(LED.Modes.STATIC,
                OFF);
        LED.update();
        telemetry.addLine("Stopping");
    }
}
