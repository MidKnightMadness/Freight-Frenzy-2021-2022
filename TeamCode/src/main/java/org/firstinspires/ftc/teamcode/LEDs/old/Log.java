package org.firstinspires.ftc.teamcode.LEDs.old;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.PrintWriter;
import java.io.File;

@Disabled
public class Log {
    public static PrintWriter out;
    public static void init() {
        if (out != null) return;
        try {
            out = new PrintWriter(new File("/storage/self/primary/FIRST/java/src/org/firstinspires/ftc/teamcode/led/log.txt"));


        } catch (Exception e) {}
    }
    // todo: write your code here
}