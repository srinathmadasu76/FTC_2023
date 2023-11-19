package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="teleop_robot")
public class BasicRobot extends OpMode
{
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;

    Servo leftClaw= null;
    Servo rightClaw= null;
    DcMotor arm= null;
    double griprPosition = .2;
    double griplPosition = .2;
    double griprDelta = .1;
    double griplDelta = .1;

    private DcMotor armLeft;
    private DcMotor armRight;

    double          clawOffset  = 0.3 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.01 ;                 // sets rate to move servo

    @Override
    public void init()
    {

        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");


        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw = hardwareMap.servo.get("leftclaw");
        rightClaw = hardwareMap.servo.get("rightclaw");
        arm = hardwareMap.dcMotor.get("arm");

        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop()
    {
        if (gamepad2.left_stick_y!=0.0) {
            arm.setPower(gamepad2.left_stick_y);
        }
        else{
            arm.setPower(0.);
        }
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

            if (gamepad2.y){
            clawOffset += CLAW_SPEED;

        }
        else if (gamepad2.a){
            clawOffset -= CLAW_SPEED;

        }

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.3, 0.3);
        leftClaw.setPosition(clawOffset);
        rightClaw.setPosition(-clawOffset);

    }
}