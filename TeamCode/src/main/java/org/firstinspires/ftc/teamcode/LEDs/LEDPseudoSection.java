package org.firstinspires.ftc.teamcode.LEDs;

public class LEDPseudoSection {
    private LEDSection[] sections;
    public LEDPseudoSection(LEDSection...sections) {
        this.sections = sections;
    }

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
        for (LEDSection section : sections) {
            section.setParam(param);
        }
    }

    public void setBrightness(double brightness) {
        for (LEDSection section : sections) {
            section.setBrightness(brightness);
        }
    }

    public void set(LED.Modes mode) {
        for (LEDSection section : sections) {
            section.set(mode);
        }
    }
    public void set(LEDColor...colors) {
        for (LEDSection section : sections) {
            section.set(colors);
        }
    }
}