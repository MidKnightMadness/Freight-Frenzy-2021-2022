package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class WestCoastDrive extends LinearOpMode{
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    public WestCoastDrive(HardwareMap hardwareMap){
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
    }

    public void drive(double xx, double yy, double rotation){
        leftMotor.setVelocity((xx - yy + rotation) * 1000);
        rightMotor.setVelocity((xx + yy + rotation) * 1000);
    }

    public void runOpMode(){
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
        }
    }
}
