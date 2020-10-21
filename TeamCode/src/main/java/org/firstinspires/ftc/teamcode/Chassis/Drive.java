package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Assembly;

public abstract class Drive  extends Assembly {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public abstract void drive(double forwards, double sideways, double turn);
}
