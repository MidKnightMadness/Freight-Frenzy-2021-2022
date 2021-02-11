package org.firstinspires.ftc.teamcode.LEDs;

public class LEDColor {
    byte[] bytes = new byte[4];
    // raw: 111(5 bit brightness) R G B
        /*public LEDColor(long raw) {
            bytes[0] = (byte) (raw & 0xFF);
            bytes[1] = (byte) (( raw >> 8) & 0xFF);
            bytes[2] = (byte) (( raw >> 16) & 0xFF);
            bytes[3] = (byte) (( raw >> 24) & 0xFF);
        }*/

    // brightness 0-31, standard hex color (0xFF00FF)
    public LEDColor(int color, int brightness) {
        bytes[0] = (byte) (brightness | 0xE0);
        bytes[1] = (byte) (color & 0xFF); // R
        bytes[2] = (byte) (( color >> 8) & 0xFF); // G
        bytes[3] = (byte) (( color >> 16) & 0xFF); // B
    }

    public LEDColor getDimmed(double brightness) {
        return new LEDColor(getColor(), (int) (getBrightness() * brightness));
    }

    public byte[] getBytes() {
        return bytes;
    }

    public int getColor() {
        return ((int) bytes[1] & 0xFF) | (((int) bytes[2] & 0xFF) << 8) | (((int) bytes[3] & 0xFF) << 16);
    }

    public int getBrightness() {
        return ((int) bytes[0]) & 0x1F;
    }
}