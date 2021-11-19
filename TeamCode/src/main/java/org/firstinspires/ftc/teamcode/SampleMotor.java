package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SampleMotor {
    DcMotorEx TestMotor;

    public SampleMotor(HardwareMap hardwareMap) {
        SampleMotor.java
        TestMotor = hardwareMap.get(DcMotorEx.class, "TEST");
    }

    public void run() {
        TestMotor.setMode(RUN_WITHOUT_ENCODERS);
        TestMotor.setPower(1);
    }
}
