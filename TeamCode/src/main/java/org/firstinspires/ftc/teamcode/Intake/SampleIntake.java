package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Config;

public class SampleIntake extends Intake{

    //declare intake motor
    private DcMotor intakeMotorL;
    private DcMotor intakeMotorR;

    //start motor
    @Override
    public void start() { intakeMotorL.setPower(1);
        intakeMotorR.setPower(-1);
    }

    //stop motor
    @Override
    public void stop() { intakeMotorL.setPower(0);
        intakeMotorR.setPower(0);
    }

    //manually set motor speed
    @Override
    public void setSpeed(double speed) { intakeMotorL.setPower(speed);
        intakeMotorR.setPower(-speed);
    }
}
