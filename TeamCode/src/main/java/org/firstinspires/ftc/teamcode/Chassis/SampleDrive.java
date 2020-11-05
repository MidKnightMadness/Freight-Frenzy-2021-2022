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
        double angle = visual.getRotation().thirdAngle;
        VectorF currentPosition = visual.getPosition();
        double positionX = currentPosition.get(0);
        double positionY = currentPosition.get(2);

        //turn bot until facing field's positive y-axis
        while(angle > 0){
            drive(0,0,-1);
            angle = visual.getRotation().thirdAngle;
        }
        while(angle < 0){
            drive(0,0,1);
            angle = visual.getRotation().thirdAngle;
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
        double angle = visual.getRotation().thirdAngle;
        VectorF currentPosition = visual.getPosition();
        double positionX = currentPosition.get(0);
        double positionY = currentPosition.get(2);
        target = visual.getPosition();
        double targetX = target.get(0);
        double targetY = target.get(2);

        //turn bot until facing field's positive y-axis
        while(angle > 0){
            drive(0,0,-1);
            angle = visual.getRotation().thirdAngle;
        }
        while(angle < 0){
            drive(0,0,1);
            angle = visual.getRotation().thirdAngle;
        }

        //while the target is in the I/IV Quadrant in relation to the bot, turn clockwise until target is on bot's x-axis or y-axis
        while((targetX > positionX && targetY > positionY) || (targetX > positionX && targetY < positionY)) {
            drive(0,0,1);
            positionX = currentPosition.get(0);
            positionY = currentPosition.get(2);
        }
        //while the target is in the II/III Quadrant in relation to the bot, turn counterclockwise until target is on bot's x-axis or y-axis
        while((targetX < positionX && targetY < positionY) || (targetX < positionX && targetY > positionY)) {
            drive(0,0,-1);
            positionX = currentPosition.get(0);
            positionY = currentPosition.get(2);
        }
        //the target is now directly in front of the bot, behind the bot, to the left of the bot, or to the right of the bot

        //move forwards if the target is in front
        while(positionY < targetY) {
            drive(1,0,0);
            positionY = currentPosition.get(2);
        }
        //move backwards if the target is in back
        while(positionY > targetY) {
            drive(-1,0,0);
            positionY = currentPosition.get(2);
        }

        //move left if the target is to the left
        while(positionX < targetX) {
            drive(0, 1, 0);
            positionX = currentPosition.get(0);
        }
        //move right if the target is to the right
        while(positionX > targetX) {
            drive(0, -1, 0);
            positionX = currentPosition.get(0);
        }
    }
}
