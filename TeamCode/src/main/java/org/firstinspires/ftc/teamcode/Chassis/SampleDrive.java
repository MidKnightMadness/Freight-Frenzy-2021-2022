package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Assembly;
import org.firstinspires.ftc.teamcode.Common.Config;
import org.firstinspires.ftc.teamcode.Visual.Visual;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class SampleDrive extends Drive{

    //declare wheel motors
    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    private final double maxVel = 2500;
    private Visual visual;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        motorFL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFL);
        motorFR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEFR);
        motorBL = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBL);
        motorBR = (DcMotorEx)hardwareMap.dcMotor.get(Config.DRIVEBR);
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
        double angle = visual.getRotation().thirdAngle; //replace with .getIMU or whatever
        VectorF currentPosition = visual.getPosition();
        double xPosition = currentPosition.get(0);
        double yPosition = currentPosition.get(2);

        //align bot to field's/target's positive y-axis
        while(angle > 0){
            drive(0,0,-1);
            angle = visual.getRotation().thirdAngle; //replace with .getIMU or whatever
        }
        while(angle < 0){
            drive(0,0,1);
            angle = visual.getRotation().thirdAngle; //replace with .getIMU or whatever
        }

        //while the target is in the I/IV Quadrant in relation to the bot, turn clockwise until facing target
        while((x > xPosition && y > yPosition) || (x > xPosition && y < yPosition)) {
            drive(0,0,1);
            xPosition = currentPosition.get(0);
        }
        //while the target is in the II/III Quadrant in relation to the bot, turn counterclockwise until facing target
        while((x < xPosition && y < yPosition) || (x < xPosition && y > yPosition)) {
            drive(0,0,-1);
            yPosition = currentPosition.get(2);
        }
        //the target is now directly in front of the bot, behind the bot, to the left of the bot, or to the right of the bot

        //move forwards if the bot is in front
        while(yPosition < y) {
            drive(1,0,0);
            yPosition = currentPosition.get(2);
        }
        //move backwards if the bot is in back
        while(yPosition > y) {
            drive(-1,0,0);
            yPosition = currentPosition.get(2);
        }

        //move left if the bot is to the left
        while(xPosition < x) {
            drive(0, 1, 0);
            xPosition = currentPosition.get(0);
        }
        //move right if the bot is to the right
        while(xPosition > x) {
            drive(0, -1,0);
            xPosition = currentPosition.get(0);
        }


    }
}
