package org.firstinspires.ftc.teamcode.LEDs;

public class LEDModes {
    // A simple mode to set the entire section to a single color.
    static class Static extends LEDMode {
        public void update() {
            for (int j = section.getBegin(); j < section.getEnd(); j++) {
                LED.leds[j] = color(j % colorsLength());
            }
        }
    }

    // A simple mode to set the entire section to a single color.
    static class Flashing extends LEDMode {
        private int delay = 0;

        public void update() {
            delay = (delay + 1) % 4;
            for (int j = section.getBegin(); j < section.getEnd(); j++) {
                LED.leds[j] = delay < 2 ? color(j % colorsLength()) : LED.Colors.OFF;
            }
        }
    }

    static class Running extends LEDMode {
        private int offset = 0;

        public void update() {
            for (int j = section.getBegin(); j < section.getEnd(); j++) {
                LED.leds[j] = color((j + offset) % colorsLength());
            }
            offset = (offset + 1) % colorsLength();
        }
    }

    static class Bouncing extends LEDMode {
        // Only use on LOWER
        private final int MIN = LED.RIGHT.getBegin();
        private final int MAX = LED.LEFT.getEnd() - 1;
        private final int ACCENT = 0;
        private final int BACKGROUND = 1;

        private int position = MIN;
        private int direction = 1;

        public void update() {
            position = position + direction;
            direction = position > MAX || position < MIN ? -direction : direction;

            for (int i = section.getBegin(); i < section.getEnd(); i++) {
                LED.leds[i] = i == position ? color(ACCENT) : color(BACKGROUND);
            }
        }
    }

    static class Progress extends LEDMode {

        private final int RMIN = LED.RIGHT.getBegin();
        private final int LMAX = LED.LEFT.getEnd() - 1;
        private final int BMAX = LED.BACK.getEnd() - 1;
        private final int RLENGTH = LED.RIGHT.getEnd() - LED.RIGHT.getBegin();
        private final int LLENGTH = LED.LEFT.getEnd() - LED.LOWER_BACK.getBegin() - 1;
        private final int ACCENT = 0;
        private final int BACKGROUND = 1;

        public void update() {
            int position = getParam();

            for (int i = section.getBegin(); i < section.getEnd(); i++) {
                LED.leds[i] = (i >= RMIN && i < position + RMIN) ||
                        i > LMAX - position ||
                        ((i < position - RLENGTH) || (i > LLENGTH - position && i <= BMAX))
                        ? color(ACCENT) : color(BACKGROUND);
            }
        }
    }
}
