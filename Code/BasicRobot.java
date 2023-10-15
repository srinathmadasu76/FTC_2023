package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="teleop_robot")
public class BasicRobot extends OpMode
{
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    @Override
    public void init()
    {

        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        if (gamepad1.left_stick_y!=0.0){
            FrontLeft.setPower(-gamepad1.left_stick_y);
            BackLeft.setPower(-gamepad1.left_stick_y);
        }
        else{
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
        }
        if (gamepad1.right_stick_y!=0.0){
            FrontRight.setPower(-gamepad1.right_stick_y);
            BackRight.setPower(-gamepad1.right_stick_y);
        }
        else{
            FrontRight.setPower(0);
            BackRight.setPower(0);
        }
        if (gamepad1.left_bumper){
            FrontLeft.setPower(-1);
            BackLeft.setPower(1);
            FrontRight.setPower(1);
            BackRight.setPower(-1);
        }
       else if(gamepad1.right_bumper){
            FrontLeft.setPower(1);
            BackLeft.setPower(-1);
            FrontRight.setPower(-1);
            BackRight.setPower(1);
        }

    }
}