package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    CRServo carouselServo;

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
}
