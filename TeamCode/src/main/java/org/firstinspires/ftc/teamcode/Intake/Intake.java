package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Intake {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public abstract void start();
    public abstract void stop();
    public abstract void setSpeed(double speed);
}
