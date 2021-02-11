package org.firstinspires.ftc.teamcode.LEDs;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.Arrays;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LEDs.old.Log;


// Control all the LEDs!!!
public class LED {

    /**** LEDColor Translation - HEX RGB and brightness to DotStar byte field ****/
    public static class Colors {
        /**** LEDColor Presets. Add new colors here. ****/
        public static final LEDColor OFF = new LEDColor(0, 0);
        public static final LEDColor RED = new LEDColor(0xFF0000, 31);
        public static final LEDColor ORANGE = new LEDColor(0xFF6300, 31);
        public static final LEDColor YELLOW = new LEDColor(0xFFFF00, 31);
        public static final LEDColor GREEN = new LEDColor(0x00FF00, 31);
        public static final LEDColor BLUE = new LEDColor(0x0000FF, 31);
        public static final LEDColor NAVY = new LEDColor(0x000090, 31);
        public static final LEDColor GOLD = new LEDColor(0xEEEE00, 31);
        public static final LEDColor PINK = new LEDColor(0xFF3030, 31);

    }

    /**** Preset modes for handling LEDs. Add more here. Assume fresh instances.****/
    public enum Modes {
        STATIC,
        RUNNING,
        BOUNCING,
        PROGRESS,
        FLASHING;

        LEDMode getNewMode() {
            switch (this) {
                case STATIC:
                default: return new LEDModes.Static();
                case RUNNING: return new LEDModes.Running();
                case BOUNCING: return new LEDModes.Bouncing();
                case PROGRESS: return new LEDModes.Progress();
                case FLASHING: return new LEDModes.Flashing();
            }
        }
    }

    /**** LED configuration. Set the lengths and make new sections as needed. Order matters. ****/
    public static final LEDSection BACK = new LEDSection(7);
    public static final LEDSection RIGHT = new LEDSection(8);
    public static final LEDSection LOWER_BACK = new LEDSection( 7);
    public static final LEDSection LEFT = new LEDSection(8);

    public static final LEDPseudoSection ALL = new LEDPseudoSection(BACK, RIGHT, LEFT, LOWER_BACK);
    public static final LEDPseudoSection LOWER = new LEDPseudoSection(RIGHT, LEFT, LOWER_BACK);



    private static I2cDeviceSynch ledController;
    static LEDColor[] leds = new LEDColor[30];
    private static boolean initialized = false;
    private static ElapsedTime runtime = new ElapsedTime();

    // Initialize the LEDs with the LED hardware.
    public static void init(I2cDeviceSynch ledStrip) {
        //Log.init();
        LED.ledController = ledStrip;//.getDevice();
        ledStrip.disengage();
        ledStrip.setI2cAddress(I2cAddr.create8bit(0x50));
        ledStrip.engage();
        for (int j = 0; j < leds.length; j++) {
            leds[j] = Colors.OFF;
        }
        ALL.set(Modes.STATIC, Colors.OFF);
        initialized = true;
    }

    private static double updateInterval = 0.1;
    public static void setUpdateInterval(double seconds) {
        updateInterval = 0.1;
    }

    public static void update() {
        if (runtime.seconds() > updateInterval)
          forceUpdate();
    }

    public static void forceUpdate()
    {
        if (!initialized) return;

        LEDSection.updateAll();
        setLEDs(leds);
        runtime.reset();
    }

    // Push the internal led buffer to the LEDs
    private static void setLEDs(LEDColor[] leds) {
        byte[] data = new byte[(leds.length + 2) * 4];
        byte[] bytes;
        int i = 0;
        for (LEDSection section : LEDSection.sections) { // an LED update frame starts with four bytes of zeroes followed by frames of four bytes for each led.
            for (; i < section.getEnd(); i++) {
                bytes = leds[i].getDimmed(section.getBrightness()).getBytes();
                data[(i * 4) + 4] = bytes[0];
                data[(i * 4) + 5] = bytes[1];
                data[(i * 4) + 6] = bytes[2];
                data[(i * 4) + 7] = bytes[3];
            }
        }

        data[data.length - 1] = data[data.length - 2] = data[data.length - 3] = data[data.length - 4] = (byte)0xFF; // and ends with four bytes of 0xFF

        //Log.out.println(Arrays.toString(data));
        for (int j = 0; j < data.length; j += 99) {
            byte[] chunk = Arrays.copyOfRange(data, j, j + 99 > data.length ? data.length : j + 99);
            //Log.out.println(Arrays.toString(chunk));
            //Log.out.println(data.length);
            //Log.out.println(j);
            //Log.out.flush();
            ledController.write(0x01, chunk);
            //Log.out.println("Sent!");
            //Log.out.println(j + 99 > data.length);
            //Log.out.flush();
            try {Thread.sleep(10);} catch (Exception e) {} // delay a tiny bit. The rev hub gets angry if you send data too fast
            if (j + 99 > data.length) break;
        }

        //Log.out.println("done!");
        //Log.out.flush();

        //ledController.write(0x01, Arrays.copyOfRange(data, 0, 99));

        // 0x01 is the function ID for SPI write
    }
}
