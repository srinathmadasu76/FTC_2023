package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="teleop_robot")
public class BasicRobot extends OpMode
{
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    DcMotor Intake = null;
    Servo leftClaw= null;
    Servo rightClaw= null;
    DcMotor arm= null;
    DcMotor hangingmotor = null;
    CRServo contServo;
    //DcMotor rackpMotor=null;
    CRServo rackpServo;
    CRServo droneServo;
    double griprPosition = .2;
    double griplPosition = .2;
    double griprDelta = .1;
    double griplDelta = .1;

    private DcMotor armLeft;
    private DcMotor armRight;
    double          clawOffset  = 0. ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.01 ;                 // sets rate to move servo
    private ElapsedTime runtime = new ElapsedTime();
    double cntPower =0.;
    double racpPower = 0.;
    double dronePower = -1;
    double armpower = 0.5;
    double hangingmotorpower=0.9;
    double motor_ticks_count = 728;
    double turn;
    int newTarget;
    @Override
    public void init()
    {

        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");

        Intake = hardwareMap.dcMotor.get("Intake");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // = hardwareMap.servo.get("leftclaw");
        //rightClaw = hardwareMap.servo.get("rightclaw");
        arm = hardwareMap.dcMotor.get("arm");
        hangingmotor = hardwareMap.dcMotor.get("hangingmotor");
        contServo = hardwareMap.crservo.get("grabberServo");
        rackpServo = hardwareMap.crservo.get("rackpinnionServo");
        droneServo = hardwareMap.crservo.get("droneServo");
        //rackpMotor = hardwareMap.dcMotor.get("rackpMotor");
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop()
    {
        Intake.setPower(0.);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        arm.setPower(0.);
        rackpServo.setPower(0. );
        //rackpMotor.setPower(0. );
        //if (gamepad1.y) {
           //arm.setPower(0.3);
           //rackpServo.setPower(0.3);
            //runtime.reset();
            //while ( (runtime.seconds() < 0.5)) {
                //telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                //telemetry.update();
            //}
            //arm.setPower(0.);
            //rackpServo.setPower(0. );
        //}
        if (gamepad1.a){

            //clawOffset -= CLAW_SPEED;
            FrontLeft.setPower(-0.3);
            BackLeft.setPower(-0.3);
            FrontRight.setPower(-0.3);
            BackRight.setPower(-0.3);
        }
        if (gamepad1.y){

            //clawOffset -= CLAW_SPEED;
            FrontLeft.setPower(0.3);
            BackLeft.setPower(0.3);
            FrontRight.setPower(0.3);
            BackRight.setPower(0.3);
        }
        if (gamepad2.left_stick_y!=0.0) {
            armpower =  Range.clip(gamepad2.left_stick_y, -0.35, 0.85);
            arm.setPower(armpower);
        }
        else{
            arm.setPower(0.);
        }
        if (gamepad2.right_stick_y!=0.0) {
            Intake.setPower(gamepad2.right_stick_y);
        }
        else{
            Intake.setPower(0.);
        }
        if (gamepad1.left_stick_y!=0.0){
            FrontRight.setPower(-gamepad1.left_stick_y);
            BackLeft.setPower(-gamepad1.left_stick_y);
        }
        else{
            FrontRight.setPower(0);
            BackLeft.setPower(0);
        }
        if (gamepad1.right_stick_y!=0.0){
            FrontLeft.setPower(-gamepad1.right_stick_y);
            BackRight.setPower(-gamepad1.right_stick_y);
        }
        else{
            FrontLeft.setPower(0);
            BackRight.setPower(0);
        }
        if (gamepad1.left_bumper){
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);
        }
       else if(gamepad1.right_bumper){
            FrontLeft.setPower(0.5);
            BackLeft.setPower(0.5);
            FrontRight.setPower(-0.5);
            BackRight.setPower(-0.5);
        }

            if (gamepad2.y){

                    //clawOffset += CLAW_SPEED;
                cntPower = 0.;
                racpPower = 0.;

        }
        else if (gamepad2.a){

                //clawOffset -= CLAW_SPEED;
                cntPower = 0.9;

            }
            else if (gamepad2.b){

                //clawOffset -= CLAW_SPEED;
                cntPower = -0.7;

            }
        if (gamepad2.left_bumper){
            racpPower = 0.4;

        }
        else if(gamepad2.right_bumper){
            racpPower = -0.4;

        }
        else{
            racpPower = 0.;

        }

        if (gamepad2.x) {
            turn = motor_ticks_count / 8;
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           // arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            newTarget = arm.getCurrentPosition() + (int) turn;
            arm.setPower(armpower);
           arm.setTargetPosition(newTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          //  runtime.reset();


            while (arm.isBusy()) {
                telemetry.addData("Turn",  arm.getCurrentPosition());
               telemetry.update();

            }
          //  contServo.setPower(cntPower);
          //  rackpServo.setPower(racpPower);
          //  arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        //if (gamepad2.dpad_down){
            //turn = motor_ticks_count/4;
            //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //newTarget = arm.getCurrentPosition() - (int)turn;
            //arm.setTargetPosition(newTarget);
            //arm.setPower(armpower);
            //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //while (gamepad2.dpad_down || arm.isBusy()) {
                //telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", turn);
                //telemetry.update();
            //}
            //arm.setPower(0.);
        //}
        if (gamepad2.dpad_down){
            droneServo.setPower(dronePower);
        } else if (gamepad2.dpad_up) {
            droneServo.setPower(0.);
        }
        if (gamepad1.x){

            hangingmotor.setPower(hangingmotorpower);

        }
        else if (gamepad1.b){

            hangingmotor.setPower(0.);

        }
        else if (gamepad1.dpad_down) {
            hangingmotor.setPower(-hangingmotorpower);
        }

            contServo.setPower(cntPower);
            rackpServo.setPower(racpPower);
            //rackpMotor.setPower(racpPower);
            //clawOffset = Range.clip(clawOffset, -0.3, 0.3);
            //leftClaw.setPosition(clawOffset);
            //rightClaw.setPosition(-clawOffset);
        }

    }
