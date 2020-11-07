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
    public void moveToPosition(double x, double y) {
        double angle = imu.getAngularOrientation().thirdAngle;
        VectorF currentPosition = visual.getPosition();
        double positionX = currentPosition.get(0);
        double positionY = currentPosition.get(2);

        //turn bot until facing field's positive y-axis
        while(angle > 0){
            drive(0,0,-1);
            angle = imu.getAngularOrientation().thirdAngle;
        }
        while(angle < 0){
            drive(0,0,1);
            angle = imu.getAngularOrientation().thirdAngle;
        }

        //while the target is in the I/IV Quadrant in relation to the bot, turn clockwise until target is on bot's x-axis or y-axis
        while((x > positionX && y > positionY) || (x > positionX && y < positionY)) {
            drive(0,0,1);
            positionX = currentPosition.get(0);
            positionY = currentPosition.get(2);
        }
        //while the target is in the II/III Quadrant in relation to the bot, turn counterclockwise until target is on bot's x-axis or y-axis
        while((x < positionX && y < positionY) || (x < positionX && y > positionY)) {
            drive(0,0,-1);
            positionX = currentPosition.get(0);
            positionY = currentPosition.get(2);
        }
        //the target is now directly in front of the bot, behind the bot, to the left of the bot, or to the right of the bot

        //move forwards if the target is in front
        while(positionY < y) {
            drive(1,0,0);
            positionY = currentPosition.get(2);
        }
        //move backwards if the target is in back
        while(positionY > y) {
            drive(-1,0,0);
            positionY = currentPosition.get(2);
        }

        //move left if the target is to the left
        while(positionX < x) {
            drive(0, 1, 0);
            positionX = currentPosition.get(0);
        }
        //move right if the target is to the right
        while(positionX > x) {
            drive(0, -1, 0);
            positionX = currentPosition.get(0);
        }
    }

    @Override
    public void moveToPosition(VectorF target) {
        double targetX = target.get(0);
        double targetY = target.get(2);

        moveToPosition(targetX, targetY);
    }

    @Override
    public void move(double inchesX, double inchesY) {
        inchesX *= encoderTicksPerInch;
        inchesY *= encoderTicksPerInch;

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setTargetPosition(motorFL.getCurrentPosition() + (int)inchesY + (int)inchesX);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - (int)inchesY + (int)inchesX);
        motorBL.setTargetPosition(motorBL.getCurrentPosition() + (int)inchesY - (int)inchesX);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() - (int)inchesY - (int)inchesX);
    }
}
