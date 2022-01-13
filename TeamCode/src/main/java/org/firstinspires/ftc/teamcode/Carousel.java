package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    CRServo cServo;

    public Carousel(HardwareMap hardwareMap){
        cServo = hardwareMap.get(CRServo.class, "carouselServo");
    }

    public void spinBlue() {
        cServo.setPower(1.0);
    }

    public void spinRed() {
        cServo.setPower(-1.0);
    }

    public void spinOff() {
        cServo.setPower(0.0);
    }
}
