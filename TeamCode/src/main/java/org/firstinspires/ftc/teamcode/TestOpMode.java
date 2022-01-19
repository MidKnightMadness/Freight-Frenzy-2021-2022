package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
/*
y (toggle) = catapult to upper
b (toggle) = catapult to middle
a (toggle) = catapult to lower
x (toggle) = intake flap
left bumper (toggle) = team shipping element motor
left trigger (toggle) = team shipping element servo
right bumper (toggle) = surgical tubing
dpad up (hold) = go to shipping hub position
dpad left (hold) = spin carousel left
dpad right (hold) = spin carousel right

*/
@TeleOp
public class TestOpMode extends OpMode {
    SampleDrive drive;
    Catapult catapult;
    Carousel carousel;
    Intake intake;
    ShippingElement shipElement;

    private DistanceSensor sensorDistanceL; //left front sensor
    private ModernRoboticsI2cRangeSensor sensorRangeM; //middle front range sensor
    private DistanceSensor sensorDistanceR; //right front sensor

    private boolean lastPressedCatapultUpper = false;
    private boolean lastPressedCatapultMiddle = false;
    private boolean lastPressedCatapultLower = false;
    private boolean catapultToggleUpper = false;
    private boolean catapultToggleMiddle = false;
    private boolean catapultToggleLower = false;

    private boolean lastPressedSurgical = false;
    private boolean surgicalToggle = false;
    private boolean lastPressedElementMotor = false;
    private boolean elementMotorToggle = false;
    private boolean lastPressedElementServo = false;
    private boolean elementServoToggle = false;
    private boolean lastPressedFlap = false;
    private boolean flapToggle = false;

    @Override
    public void init() {
        drive = new SampleDrive(hardwareMap);
        catapult = new Catapult(hardwareMap);
        carousel = new Carousel(hardwareMap);
        intake = new Intake(hardwareMap);
        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "sensor_distance_left");
        sensorRangeM = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_middle");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "sensor_distance_right");
    }

    @Override
    public void loop() {
        //drive to shipping hub position
        if ((sensorRangeM.getDistance(DistanceUnit.INCH) <= 8.5 || sensorRangeM.getDistance(DistanceUnit.INCH) >= 9.5) &&
                gamepad1.dpad_up && sensorRangeM.getDistance(DistanceUnit.INCH) < 100) {
            drive.drive(0, (sensorRangeM.getDistance(DistanceUnit.INCH) - 9) / 10, 0);
        } else {
            drive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            drive.telemetry(telemetry);
        }

        //catapult
        if (gamepad1.y && !lastPressedCatapultUpper) {
            catapultToggleUpper = !catapultToggleUpper;
            catapultToggleMiddle = false;
            catapultToggleLower = false;
        } else if (gamepad1.b && !lastPressedCatapultMiddle) {
            catapultToggleMiddle = !catapultToggleMiddle;
            catapultToggleUpper = false;
            catapultToggleLower = false;
        } else if (gamepad1.a && !lastPressedCatapultLower) {
            catapultToggleLower = !catapultToggleLower;
            catapultToggleUpper = false;
            catapultToggleMiddle = false;
        }
        if (catapultToggleUpper) {
            catapult.upper();
        } else if (catapultToggleMiddle) {
            catapult.middle();
        } else if (catapultToggleLower) {
            catapult.lower();
        } else {
            catapult.returnPosition();
        }
        lastPressedCatapultUpper = gamepad1.y;
        lastPressedCatapultMiddle = gamepad1.b;
        lastPressedCatapultLower = gamepad1.a;

        //team shipping element
        if (gamepad1.left_bumper && !lastPressedElementMotor) {
            elementMotorToggle = !elementMotorToggle;
        }
        if (elementMotorToggle) {
            shipElement.lift();
        } else {
            shipElement.lower();
        }
        lastPressedElementMotor = gamepad1.left_bumper;

        if (gamepad1.left_trigger > 0 && !lastPressedElementServo) {
            elementServoToggle = !elementServoToggle;
        }
        if (elementMotorToggle) {
            shipElement.open();
        } else {
            shipElement.close();
        }
        lastPressedElementServo = (gamepad1.left_trigger > 0);

        //surgical tubing
        if (gamepad1.right_bumper && !lastPressedSurgical) {
            surgicalToggle = !surgicalToggle;
        }
        if (surgicalToggle) {
            intake.surgicalTubingOn();
        } else {
            intake.surgicalTubingOff();
        }
        lastPressedSurgical = gamepad1.right_bumper;

        //spinning carousel
        if(gamepad1.dpad_left) {
            carousel.spinRed();
        } else if(gamepad1.dpad_right) {
            carousel.spinBlue();
        } else {
            carousel.spinOff();
        }

        //turn flap
        if(gamepad1.x && !lastPressedFlap) {
            flapToggle = !flapToggle;
        }
        if(flapToggle) {
            catapult.flapOn();
        }
        else {
            catapult.flapOff();
        }
        lastPressedFlap = gamepad1.x;
    }
}
