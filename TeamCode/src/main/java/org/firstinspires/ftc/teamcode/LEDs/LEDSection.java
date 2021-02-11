package org.firstinspires.ftc.teamcode.LEDs;

import java.util.ArrayList;

/**** Sections. Since there are four main sections of the robot, this keeps track of the mode and location of each in the led buffer ****/
public class LEDSection {
    private int begin, length; // the beginning index and the length of this section
    private static int nextLED = 0; // the next available led not claimed yet
    static ArrayList<LEDSection> sections = new ArrayList<>(); // the internal array for looping over all the sections to update
    LEDColor[] colors;
    private LEDMode mode; // Which mode this section is in. Defaults to static off.
    private LED.Modes modeEnum;
    private double brightness = 1;
    int param = 0;

    //private LEDMode lastMode;

    // Construct a section from a length
    LEDSection(int length) {
        this.begin = nextLED; // this should start at the next available led
        this.length = length; // save the length
        nextLED += length;    // and update the next available led
        sections.add(this);   // and add this to the internal array for updating
    }

    // These are made specifically for for loops. Ex: for (int i = getBegin(); i < getEnd(); i++) {}
    int getBegin() { return begin; } // for looping: returns the first index
    int getEnd() { return begin + length; } // for looping: returns the last index (exclusive)

    public void set(LED.Modes mode, int param, LEDColor...colors) {
        setParam(param);
        set(colors);
        set(mode);
    }

    public void set(LED.Modes mode, LEDColor...colors) {
        set(colors);
        set(mode);
    }

    public void set(LED.Modes mode, int param) {
        setParam(param);
        set(mode);
    }

    public void setParam(int param) {
        this.param = param;
    }

    // brightness percentage for all the colors in this section...
    public void setBrightness(double brightness) {
        this.brightness = brightness;
    }

    public double getBrightness() {
        return brightness;
    }

    public void set(LED.Modes mode) {
        //lastMode = this.mode;
        this.mode = mode.getNewMode();
        this.mode.setSection(this);
        modeEnum = mode;
    }

    public void set(LEDColor...colors) {
        this.colors = colors;
    }

    public LED.Modes getModeEnum()
    {
        return modeEnum;
    }

    private void update() {
        mode.update(); // update (pass it on to the mode to update the leds)
    }

    // Update all the sections!
    static void updateAll() {
        for (int i = 0; i < sections.size(); i++) {
            sections.get(i).update();
        }
    }
}