package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Assembly;
import org.firstinspires.ftc.teamcode.Common.Config;

public class SampleDrive extends Drive{

    //declare wheel motors
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motorFL = hardwareMap.dcMotor.get(Config.DRIVEFL);
        motorFR = hardwareMap.dcMotor.get(Config.DRIVEFR);
        motorBL = hardwareMap.dcMotor.get(Config.DRIVEBL);
        motorBR = hardwareMap.dcMotor.get(Config.DRIVEBR);
    }

    @Override
    public void drive(double forwards, double sideways, double turn) {
        motorFL.setPower(forwards + sideways + turn);
        motorFR.setPower(forwards + sideways - turn);
        motorBL.setPower(forwards - sideways + turn);
        motorBR.setPower(forwards - sideways - turn);
    }
}
