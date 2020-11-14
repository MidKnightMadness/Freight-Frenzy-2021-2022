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

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        motorFR.setVelocity((forwards + sideways + turn) * maxVel);
        motorBL.setVelocity((-forwards - sideways + turn) * maxVel);
        motorBR.setVelocity((forwards - sideways + turn) * maxVel);
    }
    @Override
    public void move(double inchesX, double inchesY) {
        double hypotenuse = Math.sqrt(Math.pow(inchesX, 2) + Math.pow(inchesY, 2));
        double movingDirection = Math.asin(inchesX/hypotenuse) + imu.getAngularOrientation().firstAngle;
        currentX += Math.sin(movingDirection) * hypotenuse;
        currentY += Math.cos(movingDirection) * hypotenuse;

        inchesX *= encoderTicksPerInch;
        inchesY *= encoderTicksPerInch;

        motorFL.setTargetPosition(motorFL.getCurrentPosition() + (int)inchesY + (int)inchesX);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - (int)inchesY + (int)inchesX);
        motorBL.setTargetPosition(motorBL.getCurrentPosition() + (int)inchesY - (int)inchesX);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() - (int)inchesY - (int)inchesX);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean motorFLTolerance = false;
        boolean motorFRTolerance = false;
        boolean motorBLTolerance = false;
        boolean motorBRTolerance = false;

        while(!motorFLTolerance || !motorFRTolerance || !motorBLTolerance || !motorBRTolerance) {
            motorFLTolerance = (motorFL.getCurrentPosition() >= motorFL.getTargetPosition() - (encoderTicksPerInch * 0.1) && motorFL.getCurrentPosition() <= motorFL.getTargetPosition() + (encoderTicksPerInch * 0.1));
            motorFRTolerance = (motorFR.getCurrentPosition() >= motorFR.getTargetPosition() - (encoderTicksPerInch * 0.1) && motorFR.getCurrentPosition() <= motorFR.getTargetPosition() + (encoderTicksPerInch * 0.1));
            motorBLTolerance = (motorBL.getCurrentPosition() >= motorBL.getTargetPosition() - (encoderTicksPerInch * 0.1) && motorBL.getCurrentPosition() <= motorBL.getTargetPosition() + (encoderTicksPerInch * 0.1));
            motorBRTolerance = (motorBR.getCurrentPosition() >= motorBR.getTargetPosition() - (encoderTicksPerInch * 0.1) && motorBR.getCurrentPosition() <= motorBR.getTargetPosition() + (encoderTicksPerInch * 0.1));

            motorFL.setPower(1);
            motorFR.setPower(1);
            motorBL.setPower(1);
            motorBR.setPower(1);
        }
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void turn(double degrees) {
        double targetAngle = imu.getAngularOrientation().firstAngle + degrees;
        boolean angleTolerance = false;

        while(!angleTolerance)  {
            angleTolerance = imu.getAngularOrientation().firstAngle >= targetAngle-5 && imu.getAngularOrientation().firstAngle <= targetAngle+5;
            telemetry.addData("imu angle", imu.getAngularOrientation().firstAngle);
            if(imu.getAngularOrientation().firstAngle > 45 + targetAngle){
                drive(0,0,-1);
            }
            else if(imu.getAngularOrientation().firstAngle < -45 + targetAngle){
                drive(0,0,1);
            }
            else if(imu.getAngularOrientation().firstAngle > targetAngle){
                drive(0,0,(imu.getAngularOrientation().firstAngle-targetAngle)/(-50));
            }
            else if(imu.getAngularOrientation().firstAngle < targetAngle) {
                drive(0,0,(imu.getAngularOrientation().firstAngle-targetAngle)/(-50));
            }
        }
    }

    @Override
    public void moveToPosition(double x, double y) {
        telemetry.addLine("Start");
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

        boolean angleTolerance = false;

        telemetry.addData("imu angle", imu.getAngularOrientation().firstAngle);

        //turn bot until facing field's positive y-axis
        turn(-imu.getAngularOrientation().firstAngle);

        telemetry.addData("dist from X", distanceFromX);
        telemetry.addData("dist from Y", distanceFromY);

        //drive to the target
        telemetry.addLine("start move");
        if(x > currentX && y > currentY) {
            move(distanceFromX, distanceFromY);
        }
        else if(x > currentX && y < currentY) {
            move(distanceFromX, -distanceFromY);
        }
        else if(x < currentX && y > currentY) {
            move(-distanceFromX, distanceFromY);
        }
        else if(x < currentX && y < currentY) {
            move(-distanceFromX, -distanceFromY);
        }
        telemetry.addLine("end move");
    }

    @Override
    public void moveToPosition(VectorF target) {
        double targetX = target.get(0);
        double targetY = target.get(2);

        moveToPosition(targetX, targetY);
    }


}
