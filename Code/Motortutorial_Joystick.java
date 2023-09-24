package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="teleop")
public class Motortutorial_Joystick extends OpMode
{
    DcMotor motor;

    @Override
    public void init()
    {
        motor = hardwareMap.dcMotor.get("motor0");
    }

    @Override
    public void loop()
    {
        if (gamepad1.left_stick_y!=0.0){
            motor.setPower(gamepad1.left_stick_y);
        }
        else{
            motor.setPower(0);
        }

    }
}