package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp
public class MainWestCoast extends LinearOpMode{
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotor leftIntake;
    DcMotor rightIntake;


    public MainWestCoast(HardwareMap hardwareMap){
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
    }

    public void drive(double xx, double yy, double rotation){
        fl.setVelocity((xx - yy + rotation) * 1000);
        fr.setVelocity((xx + yy + rotation) * 1000);
        bl.setVelocity((xx + yy + rotation) * 1000);
        br.setVelocity((xx + yy + rotation) * 1000);
    }

    public void runOpMode(){
        //drive = new WestCoastDrive(hardwareMap); // testing
        double[] power = new double[4];
        waitForStart();
        while(opModeIsActive()){
            power[0]=0; //front left
            power[1]=0; //front right
            power[2]=0; //back left
            power[3]=0; //back right

            //turn left
            if(this.gamepad1.left_stick_x<0){
                power[0]-=this.gamepad1.left_stick_x;
                power[1]+=this.gamepad1.left_stick_x;
                power[2]-=this.gamepad1.left_stick_x;
                power[3]+=this.gamepad1.left_stick_x;
            }

            //turn right
            if(this.gamepad1.left_stick_x>0){
                power[0]+=this.gamepad1.left_stick_x;
                power[1]-=this.gamepad1.left_stick_x;
                power[2]+=this.gamepad1.left_stick_x;
                power[3]-=this.gamepad1.left_stick_x;
            }

            //scooch forward
            if(this.gamepad1.left_stick_y>0){
                power[0]+=this.gamepad1.left_stick_y;
                power[1]+=this.gamepad1.left_stick_y;
                power[2]+=this.gamepad1.left_stick_y;
                power[3]+=this.gamepad1.left_stick_y;
            }

            //scooch backward
            if(this.gamepad1.left_stick_y<0){
                power[0]-=this.gamepad1.left_stick_y;
                power[1]-=this.gamepad1.left_stick_y;
                power[2]-=this.gamepad1.left_stick_y;
                power[3]-=this.gamepad1.left_stick_y;
            }

            fl.setPower(power[0]);
            fr.setPower(power[1]);
            bl.setPower(power[2]);
            br.setPower(power[3]);

            telemetry.addData("Front left: ", String.valueOf(fl.getPower()));
            telemetry.addData("Front right: ", String.valueOf(fr.getPower()));
            telemetry.addData("Back left: ", String.valueOf(bl.getPower()));
            telemetry.addData("Back right: ", String.valueOf(br.getPower()));

            telemetry.update();

            int intakeCounter= 0;
            boolean intaking = false;

            if(gamepad1.x && intakeCounter== 0){
                intakeCounter= 300;
                intaking= !intaking;
            }
            if(intakeCounter>0){
                intakeCounter=intakeCounter-1;
            }
            telemetry.addData("Intake Status", intakeCounter);
            telemetry.update();


        }
    }
}