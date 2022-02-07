package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel {
    CRServo carouselServo;
    DcMotorEx carouselMotor;

    public Carousel(HardwareMap hardwareMap){
        carouselServo = hardwareMap.get(CRServo.class, "carousel");
    }

    public void spinBlue() {
        carouselServo.setPower(1.0);
    }

    public void spinRed() {
        carouselServo.setPower(-1.0);
    }

    public void spinOff() {
        carouselServo.setPower(0.0);
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Carousel Power", carouselServo.getPower());
    }
}
