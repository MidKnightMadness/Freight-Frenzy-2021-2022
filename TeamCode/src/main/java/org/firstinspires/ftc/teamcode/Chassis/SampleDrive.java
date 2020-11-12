package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Common.Assembly;
import org.firstinspires.ftc.teamcode.Common.Config;
import org.firstinspires.ftc.teamcode.Test.PowerShot;
import org.firstinspires.ftc.teamcode.Visual.Visual;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import static org.firstinspires.ftc.teamcode.Common.Constants.encoderTicksPerInch;

public class SampleDrive extends Drive{

    //declare wheel motors
    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    private final double maxVel = 2500;
    private Visual visual;
    private double currentX = 0;
    private double currentY = 0;

    BNO055IMU imu;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motorFL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFL);
        motorFR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFR);
        motorBL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBL);
        motorBR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBR);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUParameters.loggingEnabled      = true;
        IMUParameters.loggingTag          = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
    }

    @Override
    public void drive(double forwards, double sideways, double turn) {
        motorFL.setVelocity((-forwards + sideways + turn) * maxVel);
        motorFR.setVelocity((forwards + sideways - turn) * maxVel);
        motorBL.setVelocity((-forwards - sideways + turn) * maxVel);
        motorBR.setVelocity((forwards - sideways - turn) * maxVel);
    }

    @Override
    public void move(double inchesX, double inchesY) {
        inchesX *= encoderTicksPerInch;
        inchesY *= encoderTicksPerInch;

        /*
        motorFL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() - (int)inchesY - (int)inchesX);
        */
        telemetry.addLine("Hello! I'm alive!");

        motorFL.setTargetPosition(motorFL.getCurrentPosition() + (int)inchesY + (int)inchesX);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - (int)inchesY + (int)inchesX);
        motorBL.setTargetPosition(motorBL.getCurrentPosition() + (int)inchesY - (int)inchesX);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() - (int)inchesY - (int)inchesX);

        telemetry.addLine("Still alive!");

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Still ok!");

        while(motorFL.getTargetPosition() != motorFL.getCurrentPosition() && motorFR.getTargetPosition() != motorFR.getCurrentPosition() && motorBL.getTargetPosition() != motorBL.getCurrentPosition() && motorBR.getTargetPosition() != motorBR.getCurrentPosition()) {
            motorFL.setPower(0.5);
            motorFR.setPower(0.5);
            motorBL.setPower(0.5);
            motorBR.setPower(0.5);
        }

        telemetry.addLine("Not dead!");

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        telemetry.addLine("Finished!");
    }

    @Override
    public void turn(double degrees) {
        double targetAngle = imu.getAngularOrientation().thirdAngle + degrees;

        while(targetAngle != imu.getAngularOrientation().thirdAngle) {
            if(targetAngle > imu.getAngularOrientation().thirdAngle) {
                drive(0,0,1);
            }
            else if(targetAngle < imu.getAngularOrientation().thirdAngle)  {
                drive(0,0,-1);
            }
        }

    }

    @Override
    public void moveToPosition(double x, double y) {

        double angle = imu.getAngularOrientation().thirdAngle;
        double distanceFromX = x;
        double distanceFromY = y;

        if(currentX > 0) {
            distanceFromX = Math.abs(currentX - x);
        }
        else if(currentX < 0) {
            distanceFromX  = Math.abs(x - currentX);
        }
        if(currentY > 0) {
            distanceFromY = Math.abs(currentY -  y);
        }
        else if(currentY < 0) {
            distanceFromY = Math.abs(y - currentY);
        }
        double distance = Math.sqrt(Math.pow(distanceFromX, 2) + Math.pow(distanceFromY, 2));

        //turn bot until facing field's positive y-axis
        while(angle !=  0)  {
            if(angle > 0){
                drive(0,0,-1);
                angle = imu.getAngularOrientation().thirdAngle;
            }
            else if(angle < 0){
                drive(0,0,1);
                angle = imu.getAngularOrientation().thirdAngle;
            }
        }

        //calculate the angle of the bot to the target using trigonometry
        double targetAngle = (90 - Math.asin(distanceFromY * distance));
        if(x > currentX && y < currentY) {
            targetAngle += 90;
        }
        else if(x < currentX && y > currentY) {
            targetAngle *= -1;
        }
        else if(x < currentX && y < currentY) {
            targetAngle = (-targetAngle) - 90;
        }

        //turn until the bot is facing the target
        while(imu.getAngularOrientation().thirdAngle != targetAngle) {
            if (targetAngle > 0) {
                drive(0, 0, 1);
            }
            else if (targetAngle < 0) {
                drive(0, 0, -1);
            }
        }
        //the target is now directly in front of the bot

        //drive to the target
        move(0, distance);
    }

    @Override
    public void moveToPosition(VectorF target) {
        double targetX = target.get(0);
        double targetY = target.get(2);

        moveToPosition(targetX, targetY);
    }


}
