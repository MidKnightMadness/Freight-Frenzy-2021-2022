package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class ManipulatorsMecanumChassis extends LinearOpMode{
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotor leftIntake;
    DcMotor rightIntake;

    public ManipulatorsMecanumChassis(HardwareMap hardwareMap){
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
    }

    public void drive(double xx, double yy, double rotation){
        leftMotor.setVelocity((xx - yy + rotation) * 1000);
        rightMotor.setVelocity((xx + yy + rotation) * 1000);
    }

    public void runOpMode(){
        //drive = new WestCoastDrive(hardwareMap); // testing
        double[] power = new double[2];
        waitForStart();
        while(opModeIsActive()){
            power[0]=0; //left
            power[1]=0; //right

            //turn left
            if(this.gamepad1.left_stick_x<0){
                power[0]-=this.gamepad1.left_stick_x;
                power[1]+=this.gamepad1.left_stick_x;
            }

            //turn right
            if(this.gamepad1.left_stick_x>0){
                power[0]+=this.gamepad1.left_stick_x;
                power[1]-=this.gamepad1.left_stick_x;
            }

            //scooch forward
            if(this.gamepad1.left_stick_y>0){
                power[0]+=this.gamepad1.left_stick_y;
                power[1]+=this.gamepad1.left_stick_y;
            }

            //scooch backward
            if(this.gamepad1.left_stick_y<0){
                power[0]-=this.gamepad1.left_stick_y;
                power[1]-=this.gamepad1.left_stick_y;
            }

            leftMotor.setPower(power[0]);
            rightMotor.setPower(power[1]);

            telemetry.addData("Left motor: ", String.valueOf(leftMotor.getPower()));
            telemetry.addData("Right motor: ", String.valueOf(rightMotor.getPower()));
            telemetry.update();
        }
    }
}