package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShippingElement {
    private DcMotor shippingElementMotor;
    private Servo shippingElementServo;
    private int motorStartPosition;

    public ShippingElement(HardwareMap hardwareMap) {
        shippingElementServo = hardwareMap.get(Servo.class, "ship_element_servo");
        shippingElementMotor = hardwareMap.get(DcMotor.class, "ship_element_motor");
        motorStartPosition = shippingElementMotor.getCurrentPosition();
    }

    public void lift() {
        shippingElementMotor.setTargetPosition(300 + motorStartPosition);
        shippingElementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shippingElementMotor.setPower(1.0);
    }

    public void lower() {
        shippingElementMotor.setTargetPosition(motorStartPosition);
        shippingElementMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shippingElementMotor.setPower(1.0);
    }

    public void open() {
        shippingElementServo.setPosition(100);
    }

    public void close() {
        shippingElementServo.setPosition(0);
    }
}
