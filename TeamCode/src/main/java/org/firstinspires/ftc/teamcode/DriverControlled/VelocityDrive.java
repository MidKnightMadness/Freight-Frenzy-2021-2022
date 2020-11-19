package org.firstinspires.ftc.teamcode.DriverControlled;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Common.Config;

@TeleOp
public class VelocityDrive extends OpMode {
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx BL;
    private DcMotorEx BR;

    BNO055IMU imu;

    @Override
    public void init() {
        FL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFL);
        FR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFR);
        BL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBL);
        BR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBR);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    public void loop() {
        FL.setVelocity((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 2500);
        FR.setVelocity((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * 2500);
        BL.setVelocity((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 2500);
        BR.setVelocity((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * 2500);

        telemetry.addData("FL vel", FL.getVelocity());
        telemetry.addData("FR vel", FR.getVelocity());
        telemetry.addData("BL vel", BL.getVelocity());
        telemetry.addData("BR vel", BR.getVelocity());
        telemetry.addData("FL pos", FL.getCurrentPosition());
        telemetry.addData("FR pos", FR.getCurrentPosition());
        telemetry.addData("BL pos", BL.getCurrentPosition());
        telemetry.addData("BR pos", BR.getCurrentPosition());
        telemetry.addData("imu angle", imu.getAngularOrientation().firstAngle);
    }
}
