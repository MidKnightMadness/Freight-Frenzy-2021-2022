package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@TeleOp
@Disabled
public class DigitalDeviceTest extends LinearOpMode {
    DigitalChannel sensorIn;
    DigitalChannel sensorOut;
    boolean lastRead;
    double startHigh;
    double endHigh;

    @Override
    public void runOpMode() {
        sensorIn = hardwareMap.digitalChannel.get("sensorIn");
        sensorIn.setMode(DigitalChannel.Mode.INPUT);
        sensorOut = hardwareMap.digitalChannel.get("sensorOut");
        sensorOut.setMode(DigitalChannel.Mode.OUTPUT);

        sensorIn.setState(true);
        sleep(1);
        sensorIn.setState(false);
        while (!isStarted() && !isStopRequested()) {
            if(sensorIn.getState() && !lastRead)
                startHigh = time;
            else if(!sensorIn.getState() && lastRead)
                endHigh = time;

            lastRead = sensorIn.getState();

            telemetry.addData("sensor reading", lastRead);
            telemetry.addData("start high", startHigh);
            telemetry.addData("end high", endHigh);
            telemetry.update();
        }
    }
}
