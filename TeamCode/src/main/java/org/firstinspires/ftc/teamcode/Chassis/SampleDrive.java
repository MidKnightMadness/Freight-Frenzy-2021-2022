package org.firstinspires.ftc.teamcode.Chassis;

import android.view.Display;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Common.Assembly;
import org.firstinspires.ftc.teamcode.Common.Config;
import org.firstinspires.ftc.teamcode.Test.PowerShot;
import org.firstinspires.ftc.teamcode.Visual.Visual;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.concurrent.Callable;

import static org.firstinspires.ftc.teamcode.Common.Constants.encoderTicksPerInch;

public class SampleDrive extends Drive{

    //declare wheel motors
    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    ModernRoboticsI2cRangeSensor distL;
    ModernRoboticsI2cRangeSensor distR;
    ModernRoboticsI2cRangeSensor distF;

    private double startDistL;
    private double startDistR;
    private final double maxVel = 2500;
    private Visual visual;
    public double currentX = 0;
    public double currentY = 0;
    public double lastFL = 0;
    public double lastFR = 0;
    public double lastBL = 0;
    public double lastBR = 0;
    public double currentAngle = 0;

    BNO055IMU imu;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        super.init(hardwareMap, telemetry, gamepad1, gamepad2);
        distL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORLEFT);
        distR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORRIGHT);
        distF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, Config.RANGESENSORFRONT);

        startDistL = distL.getDistance(DistanceUnit.INCH);
        startDistR = distR.getDistance(DistanceUnit.INCH);

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
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
    }

    //positive forwards value moves backwards, negative forwards value moves forwards
    //positive sideways value moves right, negative sideways value moves left
    //positive turn value moves clockwise, negative turn value moves counter-clockwise
    @Override
    public void drive(double forwards, double sideways, double turn) {
        motorFL.setVelocity((-forwards + sideways + turn) * maxVel);
        motorFR.setVelocity((forwards + sideways + turn) * maxVel);
        motorBL.setVelocity((-forwards - sideways + turn) * maxVel);
        motorBR.setVelocity((forwards - sideways + turn) * maxVel);

        double changeFL = motorFL.getCurrentPosition() - lastFL;
        double changeFR = motorFR.getCurrentPosition() - lastFR;
        double changeBL = motorBL.getCurrentPosition() - lastBL;
        double changeBR = motorBR.getCurrentPosition() - lastBR;

        double rotation = (changeFL + changeFR + changeBL + changeBR) / 4;
        changeFL -= rotation;
        changeFR -= rotation;
        changeBL -= rotation;
        changeBR -= rotation;

        double distanceX = -(-changeFL - changeFR + changeBL + changeBR) / 180;
        double distanceY = (changeFL - changeFR + changeBL - changeBR) / 180;

        updateAngle();
        updatePosition();
        setCurrentX(-distanceY * Math.sin(Math.toRadians(currentAngle)) + distanceX * Math.cos(Math.toRadians(currentAngle)));
        setCurrentY(distanceY * Math.cos(Math.toRadians(currentAngle)) + distanceX * Math.sin(Math.toRadians(currentAngle)));
    }

    @Override
    public void moveV2(double inchesX, double inchesY, double power) {
        updateAngle();

        double distance = Math.sqrt(inchesX*inchesX + inchesY*inchesY);
        setCurrentX((-inchesX * 0.3) * Math.sin(Math.toRadians(currentAngle)) + (inchesX * 0.3) * Math.cos(Math.toRadians(currentAngle)));
        setCurrentY((inchesY * 0.3) * Math.cos(Math.toRadians(currentAngle)) + (inchesY * 0.3) * Math.sin(Math.toRadians(currentAngle)));

        //convert to encoder ticks for run to position
        inchesX *= encoderTicksPerInch;
        inchesY *= encoderTicksPerInch;

        //set where bot should end up according to encoders
        motorFL.setTargetPosition(motorFL.getCurrentPosition() + (int)inchesY + (int)inchesX);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - (int)inchesY + (int)inchesX);
        motorBL.setTargetPosition(motorBL.getCurrentPosition() + (int)inchesY - (int)inchesX);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() - (int)inchesY - (int)inchesX);

        //add tolerance of 0.1 inches in case bot is not exact
        motorFL.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorFR.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorBL.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorBR.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double initX = currentX;
        double initY = currentY;

        telemetry.addData("initposx", currentX);
        telemetry.addData("initposy", currentY);

        while((motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy())) {
            try {
                if(isStopRequested.call())
                    return;
            }
            catch (NullPointerException exception){
                telemetry.addLine("You need to set isStopRequested when using move");
            }
            catch (Exception ignored) {}

            double prevx = currentX; //previous x
            double prevy = currentY; //previous y
            double prevAng = imu.getAngularOrientation().firstAngle; //previous angle

            //move different speeds depending on how far you're moving
            if (distance > 24) {
                motorFL.setPower(power);
                motorFR.setPower(power);
                motorBL.setPower(power);
                motorBR.setPower(power);
            } else if (distance <= 24) {
                motorFL.setPower(power / 2);
                motorFR.setPower(power / 2);
                motorBL.setPower(power / 2);
                motorBR.setPower(power / 2);
            }
            double changeFL = motorFL.getCurrentPosition() - lastFL;
            double changeFR = motorFR.getCurrentPosition() - lastFR;
            double changeBL = motorBL.getCurrentPosition() - lastBL;
            double changeBR = motorBR.getCurrentPosition() - lastBR;

            double rotation = (changeFL + changeFR + changeBL + changeBR) / 4;
            changeFL -= rotation;
            changeFR -= rotation;
            changeBL -= rotation;
            changeBR -= rotation;

            double distanceX = -(-changeFL - changeFR + changeBL + changeBR) / 180;
            double distanceY = (changeFL - changeFR + changeBL - changeBR) / 180;

            updatePosition();
            updateAngle();
            setCurrentX(-distanceY * Math.sin(Math.toRadians(currentAngle)) + distanceX * Math.cos(Math.toRadians(currentAngle)));
            setCurrentY(distanceY * Math.cos(Math.toRadians(currentAngle)) + distanceX * Math.sin(Math.toRadians(currentAngle)));

            if(currentX != prevx || currentY != prevy || currentAngle != prevAng) { //Something changed
                double dx = currentX-prevx; //change in x
                double dy = currentY-prevy; //change in y
                //double da = currentAngle-prevAng; //change in angle, not used currently but could be helpful?
                double dist = Math.sqrt(dx*dx+dy*dy); //distance traveled
                double x2 = Math.sin(-Math.toRadians(currentAngle))*dist; //x position based on gyro
                double y2 = Math.cos(-Math.toRadians(currentAngle))*dist; //y position based on gyro
                double ddx = x2-dx; //Distance along x that we are off by
                double ddy = y2-dy; //Distance along y that we are off by;

                prevx = currentX; //update previous x
                prevy = currentY; //update previous y

                inchesX += ddx-dx; //offset by error
                inchesY += ddy-dy; //offset by error

                //Update target positions!
                motorFL.setTargetPosition(motorFL.getCurrentPosition() + (int) inchesY + (int) inchesX);
                motorFR.setTargetPosition(motorFR.getCurrentPosition() - (int) inchesY + (int) inchesX);
                motorBL.setTargetPosition(motorBL.getCurrentPosition() + (int) inchesY - (int) inchesX);
                motorBR.setTargetPosition(motorBR.getCurrentPosition() - (int) inchesY - (int) inchesX);

                telemetry.addData("currentX: ", currentX);
                telemetry.addData("currentY: ", currentY);
                telemetry.addData("ddx: ", ddx);
                telemetry.addData("ddy: ", ddy);
                telemetry.addData("dx: ", dx);
                telemetry.addData("du: ", dx);
                telemetry.addData("inchesX: ", inchesX);
                telemetry.addData("inchesY: ", inchesY);
            }
            telemetry.addData("Gryo Angle: ", imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }

        //stop everything
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
    public void move(double inchesX, double inchesY, double power) {
        updateAngle();

        double distance = Math.sqrt(Math.pow(inchesX, 2) + Math.pow(inchesY, 2));
        setCurrentX((-inchesX * 0.3) * Math.sin(Math.toRadians(currentAngle)) + (inchesX * 0.3) * Math.cos(Math.toRadians(currentAngle)));
        setCurrentY((inchesY * 0.3) * Math.cos(Math.toRadians(currentAngle)) + (inchesY * 0.3) * Math.sin(Math.toRadians(currentAngle)));

        //convert to encoder ticks for run to position
        inchesX *= encoderTicksPerInch;
        inchesY *= encoderTicksPerInch;

        //set where bot should end up according to encoders
        motorFL.setTargetPosition(motorFL.getCurrentPosition() + (int)inchesY + (int)inchesX);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - (int)inchesY + (int)inchesX);
        motorBL.setTargetPosition(motorBL.getCurrentPosition() + (int)inchesY - (int)inchesX);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() - (int)inchesY - (int)inchesX);

        //add tolerance of 0.1 inches in case bot is not exact
        motorFL.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorFR.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorBL.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorBR.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy())) {
            try {
                if(isStopRequested.call())
                    return;
            }
            catch (NullPointerException exception){
                telemetry.addLine("You need to set isStopRequested when using move");
            }
            catch (Exception ignored) {}

            //move different speeds depending on how far you're moving
            if (distance > 24) {
                motorFL.setPower(power);
                motorFR.setPower(power);
                motorBL.setPower(power);
                motorBR.setPower(power);
            } else if (distance <= 24) {
                motorFL.setPower(power / 2);
                motorFR.setPower(power / 2);
                motorBL.setPower(power / 2);
                motorBR.setPower(power / 2);
            }
        }
        updateAngle();

        //stop everything
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
    public void smoothMove(double inchesX, double inchesY) {
        updateAngle();
        setCurrentX((-inchesX * 0.3) * Math.sin(Math.toRadians(currentAngle)) + (inchesX * 0.3) * Math.cos(Math.toRadians(currentAngle)));
        setCurrentY((inchesY * 0.3) * Math.cos(Math.toRadians(currentAngle)) + (inchesY * 0.3) * Math.sin(Math.toRadians(currentAngle)));

        //convert to encoder ticks for run to position
        inchesX *= encoderTicksPerInch;
        inchesY *= encoderTicksPerInch;

        //set where bot should end up according to encoders
        motorFL.setTargetPosition(motorFL.getCurrentPosition() + (int)inchesY + (int)inchesX);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - (int)inchesY + (int)inchesX);
        motorBL.setTargetPosition(motorBL.getCurrentPosition() + (int)inchesY - (int)inchesX);
        motorBR.setTargetPosition(motorBR.getCurrentPosition() - (int)inchesY - (int)inchesX);

        //add tolerance of 0.1 inches in case bot is not exact
        motorFL.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorFR.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorBL.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));
        motorBR.setTargetPositionTolerance((int)(encoderTicksPerInch * 0.3));

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double flGoal = Math.abs(motorFL.getCurrentPosition() - motorFL.getTargetPosition());
        double frGoal = Math.abs(motorFR.getCurrentPosition() - motorFR.getTargetPosition());
        double blGoal = Math.abs(motorBL.getCurrentPosition() - motorBL.getTargetPosition());
        double brGoal = Math.abs(motorBR.getCurrentPosition() - motorBR.getTargetPosition());

        double goal = (flGoal + frGoal + blGoal + brGoal) / 4;

        while((motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy())) {
            try {
                if(isStopRequested.call())
                    return;
            }
            catch (NullPointerException exception){
                telemetry.addLine("You need to set isStopRequested when using move");
            }
            catch (Exception ignored) {}

            double flProgress = Math.abs(motorFL.getCurrentPosition() - motorFL.getTargetPosition());
            double frProgress = Math.abs(motorFR.getCurrentPosition() - motorFR.getTargetPosition());
            double blProgress = Math.abs(motorBL.getCurrentPosition() - motorBL.getTargetPosition());
            double brProgress = Math.abs(motorBR.getCurrentPosition() - motorBR.getTargetPosition());

            double progress = (flProgress + frProgress + blProgress + brProgress) / 4;
            if(progress == goal) {
                motorFL.setPower(0.1);
                motorFR.setPower(0.1);
                motorBL.setPower(0.1);
                motorBR.setPower(0.1);
            }
            //speed up for first portion
            else if((progress/goal) > 0.25) {
                motorFL.setPower(1 - (progress / goal));
                motorFR.setPower(1 - (progress / goal));
                motorBL.setPower(1 - (progress / goal));
                motorBR.setPower(1 - (progress / goal));
            }
            //slow down for second portion
            else {
                motorFL.setPower(progress / goal);
                motorFR.setPower(progress / goal);
                motorBL.setPower(progress / goal);
                motorBR.setPower(progress / goal);
            }

        }
        updateAngle();

        //stop everything
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
    public void moveV2(double inchesX, double inchesY) { moveV2(inchesX, inchesY, 1 );}

    //just move without a power specified
    @Override
    public void move(double inchesX, double inchesY) {
        move(inchesX, inchesY, 1);
    }

    //adjust to point (x, y)
    @Override
    public void adjust(double x, double y) {
        double targetF = 127 - y;
        double targetR = 15 - x;
        //double targetL = 65 + x;

        double distOffF = 1;
        double distOffR = 1;
        //double distOffL = 1;
        double turn = 1;

        while (Math.abs(distOffF) > 0.3 || /*Math.abs(distOffL) > 0.3 ||*/ Math.abs(distOffR) > 0.3 || Math.abs(turn) > 0.033333) {
            distOffF = distF.getDistance(DistanceUnit.INCH);
            if (distOffF != 0) {
                distOffF = -(distOffF - targetF) / 48;
            }
            distOffR = (distR.getDistance(DistanceUnit.INCH) - targetR) / 48;
            //distOffL = -(distL.getDistance(DistanceUnit.INCH) - targetL) / 48;
            turn = getAngle() / 30;

            //discard unusual output
            if (Math.abs(distOffR) > 1000)
                distOffR = 0;
            //if (Math.abs(distOffL) > 1000)
            //    distOffL = 0;
            if (Math.abs(distOffF) > 1000)
                distOffF = 0;

            if(distOffF < 0) {
                distOffF = 0;
            }
            
            //if (distOffR < distOffL) {
                drive(distOffF, distOffR, turn);
            //}  else if (distOffL < distOffR) {
                //drive(distOffF, distOffL, turn);
            //}
        }

        drive(0,0,0);
    }

    @Override
    public void adjustWalls(double inchesF, double inchesR) {
        double distOffF = 1;
        double distOffR = 1;
        double turn = 1;

        while (Math.abs(distOffF) >  0.3 || Math.abs(distOffR) > 0.3 || Math.abs(turn) > 0.033333) {
            distOffF = distF.getDistance(DistanceUnit.INCH);
            if (distOffF != 0) {
                distOffF = -(distOffF - inchesF) / 48;
            }
            distOffR = (distR.getDistance(DistanceUnit.INCH) - inchesR) / 48;
            turn = getAngle() / 30;

            //discard unusual output
            if (Math.abs(distOffR) > 1000)
                distOffR = 0;
            if (Math.abs(distOffF) > 1000)
                distOffF = 0;

            if(distOffF < 0) {
                distOffF = 0;
            }

            drive(distOffF, distOffR, turn);
        }

        drive(0,0,0);
    }

    //imu only accounts for angles from -180 to 180
    //convert angles out of this range to usable angles in this range
    public double convertAngle(double degrees) {
        if(degrees > 180) {
            return degrees - 360;
        }
        else if(degrees < -180) {
            return degrees + 360;
        }
        else{
            return degrees;
        }
    }

    public int convertAngle(int degrees) {
        if(degrees > 180) {
            return degrees - 360;
        }
        else if(degrees < -180) {
            return degrees + 360;
        }
        else{
            return degrees;
        }
    }


    //positive degrees is counter clockwise and negative degrees is clockwise
    @Override
    public void turn(double degrees) {
        updateAngle();
        double targetAngle = convertAngle(currentAngle + degrees);
        int angleCounter;
        double clockwiseDistance = 0;
        double counterClockwiseDistance = 0;

        boolean angleTolerance = false;

        while(!angleTolerance)  {
            try {
                if(isStopRequested.call())
                    return;
            }
            catch (NullPointerException exception){
                telemetry.addLine("You need to set isStopRequested when using move");
            }
            catch (Exception ignored) {}

            updateAngle();
            //add tolerance of 3 degrees over and under in case bot is not exact
            angleTolerance = (currentAngle >= convertAngle(targetAngle-3) && currentAngle <= convertAngle(targetAngle+3));


            angleCounter = (int)currentAngle;
            //calculate clockwise distance and counter-clockwise distance
            while(angleCounter != (int)targetAngle) {
                try {
                    if(isStopRequested.call())
                        return;
                }
                catch (NullPointerException exception){
                    telemetry.addLine("You need to set isStopRequested when using move");
                }
                catch (Exception ignored) {}
                angleCounter = convertAngle(angleCounter + 1);
                counterClockwiseDistance = counterClockwiseDistance + 1;
            }

            angleCounter = (int)currentAngle;
            while(angleCounter != (int)targetAngle) {
                try {
                    if(isStopRequested.call())
                        return;
                }
                catch (NullPointerException exception){
                    telemetry.addLine("You need to set isStopRequested when using move");
                }
                catch (Exception ignored) {}
                angleCounter = convertAngle(angleCounter - 1);
                clockwiseDistance = clockwiseDistance + 1;
            }

            //turn clockwise or counter-clockwise depending on which turn will be shorter
            //turn speed is also determined by how far away it is from its target angle
            if(clockwiseDistance > counterClockwiseDistance && counterClockwiseDistance > 90){
                drive(0,0,-0.75);
            }
            else if(clockwiseDistance < counterClockwiseDistance && clockwiseDistance > 90){
                drive(0,0,0.75);
            }
            else if(clockwiseDistance > counterClockwiseDistance){
                drive(0,0,-0.5);
            }
            else if(clockwiseDistance < counterClockwiseDistance) {
                drive(0,0,0.5);
            }
            updateAngle();
        }
        //stop everything
        telemetry.addLine("turning done");
        drive(0,0,0);
        telemetry.update();
    }

    //positive degrees is counter clockwise and negative degrees is clockwise
    @Override
    public void betterTurn(double degrees) {
        updateAngle();
        double targetAngle = convertAngle(currentAngle + degrees);
        double angleDifference = 1000;

        while(angleDifference > 1)  {
            try {
                if(isStopRequested.call())
                    return;
            }
            catch (NullPointerException exception){
                telemetry.addLine("You need to set isStopRequested when using move");
            }
            catch (Exception ignored) {}

            updateAngle();
            angleDifference = Math.abs(currentAngle - targetAngle);

            if(targetAngle > currentAngle) {
                drive(0,0, -angleDifference);
            }
            else if(currentAngle > targetAngle) {
                drive(0,0, angleDifference);
            }
        }
        //stop everything
        telemetry.addLine("turning done");
        drive(0,0,0);
        telemetry.update();
    }

    @Override
    public void turnToPoint(double x, double y) {
        updateAngle();
        double targetAngle = Math.toDegrees(Math.atan2(y - currentY, x - currentX));  //get the angle that we want to turn to
        betterTurn(targetAngle - currentAngle);  //turn the amount of offset
    }

    //turn until the bot is facing the front of the field
    //technically just turns the bot whatever angle it faced when the round started
    @Override
    public void alignForward() {
        updateAngle();
        betterTurn(-currentAngle);
    }

    //move the bot to a position on the field using maths
    @Override
    public void moveToPosition(double x, double y, double power) {
        //determine how faw away the bot is from where you want it to go
        double distanceFromX = Math.abs(x);
        double distanceFromY = Math.abs(y);

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

        //turn bot until facing field's positive y-axis
        alignForward();

        //drive to the target
        if(x > currentX && y > currentY) {
            move(distanceFromX, distanceFromY, power);
        }
        else if(x > currentX && y < currentY) {
            move(distanceFromX, -distanceFromY, power);
        }
        else if(x < currentX && y > currentY) {
            move(-distanceFromX, distanceFromY, power);
        }
        else if(x < currentX && y < currentY) {
            move(-distanceFromX, -distanceFromY, power);
        }

        //turn bot facing forwards again
        alignForward();

        telemetry.addData("x position", currentX);
        telemetry.addData("y position", currentY);
        telemetry.update();
    }

    @Override
    public void moveToPosition(double x, double y) {
        moveToPosition(x, y,1);
    }

    //same thing as moveToPosition(double x, double y) but extracts target position from a vector
    @Override
    public void moveToPosition(VectorF target) {
        double targetX = target.get(0);
        double targetY = target.get(2);

        moveToPosition(targetX, targetY);
    }

    //move the bot to the shooting position in front of the tower
    @Override
    public void moveToTower() {
        //the shooting position is different depending on where the bot starts in the round
        //use distance sensors to determine where we are starting from to determine the shooting position
        if(startDistL < 48 && startDistR > 24) {
            //left line
            moveToPosition(-3.75,50);
        }
        else {
            //right line
            moveToPosition(-28.75, 50);
        }
    }

    //move the bot to the shooting position in front of the leftmost powershot target
    @Override
    public void moveToPower1() {
        //the shooting position is different depending on where the bot starts in the round
        //use distance sensors to determine where we are starting from to determine the shooting position
        if(startDistL < 48 && startDistR > 24) {
            moveToPosition(-35.25,50);
        }
        else {
            moveToPosition(-60.25,50);
        }
    }

    //move the bot to the shooting position in front of the center powershot target
    @Override
    public void moveToPower2() {
        //the shooting position is different depending on where the bot starts in the round
        //use distance sensors to determine where we are starting from to determine the shooting position
        if(startDistL < 48 && startDistR > 24) {
            //left line
            moveToPosition(-27.75,50);
        }
        else {
            //right line
            moveToPosition(-52.75,50);
        }
    }

    //move the bot to the shooting position in front of the rightmost powershot target
    @Override
    public void moveToPower3() {

        //the shooting position is different depending on where the bot starts in the round
        //use distance sensors to determine where we are starting from to determine the shooting position
        if(startDistL < 48 && startDistR > 24) {
            //left line
            moveToPosition(-20.25,50);
        }
        else {
            //right line
            moveToPosition(-45.25,50);
        }
    }

    //stop everything
    @Override
    public void stop()
    {
        motorBR.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
    }

    //get the angle
    @Override
    public double getAngle() {
        currentAngle = imu.getAngularOrientation().firstAngle;
        return currentAngle;
    }

    //get x-position
    @Override
    public double getCurrentX() {
        return currentX;
    }

    //get y-position
    @Override
    public double getCurrentY() {
        return currentY;
    }

    //set the angle
    @Override
    public void setAngle(double angle) {
        currentAngle = angle;
    }

    //set x-position
    @Override
    public void setCurrentX(double x) {
        currentX = x;
    }

    //set y-position
    @Override
    public void setCurrentY(double y) {
        currentY = y;
    }

    @Override
    public void updatePosition() {
        lastFL = motorFL.getCurrentPosition();
        lastFR = motorFR.getCurrentPosition();
        lastBL = motorBL.getCurrentPosition();
        lastBR = motorBR.getCurrentPosition();
    }

    //update angular position
    @Override
    public void updateAngle() {
        setAngle(imu.getAngularOrientation().firstAngle);
    }
}
