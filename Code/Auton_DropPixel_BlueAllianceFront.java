package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Autonomous(name="Auton_DropPixel_BlueAllianceFront", group="Robot")
//@Disabled
public class Auton_DropPixel_BlueAllianceFront extends LinearOpMode {
    private DistanceSensor sensorleft;
    private DistanceSensor sensorright;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    //DcMotor Intake = null;
    DcMotor arm= null;
    CRServo racpServo;
    private ElapsedTime runtime = new ElapsedTime();
    double racpPower = 0.3;
    double armPower = 0.5;

    double desiredPosition = 10;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorleft = hardwareMap.get(DistanceSensor.class, "sensor_left");
        sensorright = hardwareMap.get(DistanceSensor.class, "sensor_right");
        FrontLeft = hardwareMap.dcMotor.get("FL");
        FrontRight = hardwareMap.dcMotor.get("FR");
        BackLeft = hardwareMap.dcMotor.get("BL");
        BackRight = hardwareMap.dcMotor.get("BR");
        //Intake = hardwareMap.dcMotor.get("Intake");
        arm = hardwareMap.dcMotor.get("arm");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // = hardwareMap.servo.get("leftclaw");
        //rightClaw = hardwareMap.servo.get("rightclaw");

        racpServo = hardwareMap.crservo.get("grabberServo");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)  sensorleft;
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)  sensorright;
        waitForStart();


        FrontLeft.setPower(0.3);
        BackLeft.setPower(0.3);
        FrontRight.setPower(-0.3);
        BackRight.setPower(-0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(-0.3);
        BackLeft.setPower(-0.3);
        FrontRight.setPower(-0.3);
        BackRight.setPower(-0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        double valueright = sensorright.getDistance(DistanceUnit.INCH);
        double valueleft = sensorleft.getDistance(DistanceUnit.INCH);
        //while (opModeIsActive() && (runtime.seconds() < 3.8)) {
        //telemetry.addData("Distance-right", "Leg 2: %4.1f S Elapsed", valueright);
        // telemetry.update();
        // }
        if (valueleft > 1 && valueleft < 14) {
            FrontLeft.setPower(-0.15);
            BackLeft.setPower(-0.15);
            FrontRight.setPower(-0.15);
            BackRight.setPower(-0.15);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.8)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }
            FrontLeft.setPower(0.15);
            BackLeft.setPower(0.15);
            FrontRight.setPower(-0.15);
            BackRight.setPower(-0.15);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }

            runtime.reset();
            arm.setPower(0.4);
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }

            sleep(2000);
            //Go to Park
            FrontLeft.setPower(0.5);
            BackLeft.setPower(0.5);
            FrontRight.setPower(-0.5);
            BackRight.setPower(-0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            //spin anticlockwise
            runtime.reset();
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(-0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            //double error = (desiredPosition - arm.getCurrentPosition());
            //double kp = 1;
            //armPower += kp*error;
            //if (error<0) {
            // armPower = -armPower;
            // }
            arm.setPower(armPower);
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            runtime.reset();
            racpServo.setPower(racpPower);
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            //go back
            runtime.reset();
            FrontLeft.setPower(0.2);
            BackLeft.setPower(0.2);
            FrontRight.setPower(0.2);
            BackRight.setPower(0.2);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.2)) {
                telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

        }

        else {

            FrontLeft.setPower(-0.3);
            BackLeft.setPower(-0.3);
            FrontRight.setPower(-0.3);
            BackRight.setPower(-0.3);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            //valueright = sensorright.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distanceright", "Leg 2: %4.1f S Elapsed", valueright);
            telemetry.update();
            if (valueright > 1 && valueright < 14) {
                FrontLeft.setPower(-0.15);
                BackLeft.setPower(-0.15);
                FrontRight.setPower(-0.15);
                BackRight.setPower(-0.15);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3)) {
                    telemetry.addData("Distanceright", "Leg 2: %4.1f S Elapsed", valueright);
                    telemetry.update();
                }
                FrontLeft.setPower(0.3);
                BackLeft.setPower(0.3);
                FrontRight.setPower(-0.3);
                BackRight.setPower(-0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                    telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                runtime.reset();
                arm.setPower(0.4);
                while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                    telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                sleep(2000);
                //Go to Park
                FrontLeft.setPower(0.5);
                BackLeft.setPower(0.5);
                FrontRight.setPower(-0.5);
                BackRight.setPower(-0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //spin anticlockwise
                runtime.reset();
                FrontLeft.setPower(-0.5);
                BackLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackRight.setPower(-0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.4)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //double error = (desiredPosition - arm.getCurrentPosition());
                //double kp = 1;
                //armPower += kp*error;
                //if (error<0) {
                // armPower = -armPower;
                // }
                arm.setPower(armPower);
                while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                runtime.reset();
                racpServo.setPower(racpPower);
                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //go back
                runtime.reset();
                FrontLeft.setPower(0.2);
                BackLeft.setPower(0.2);
                FrontRight.setPower(0.2);
                BackRight.setPower(0.2);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.2)) {
                    telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

            }
            else {
                FrontLeft.setPower(-0.3);
                BackLeft.setPower(-0.3);
                FrontRight.setPower(0.3);
                BackRight.setPower(0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                runtime.reset();
                arm.setPower(0.4);
                while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                    telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                sleep(2000);
                //Go to Park
                FrontLeft.setPower(0.5);
                BackLeft.setPower(0.5);
                FrontRight.setPower(-0.5);
                BackRight.setPower(-0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //spin anticlockwise
                runtime.reset();
                FrontLeft.setPower(-0.5);
                BackLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackRight.setPower(-0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.4)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //double error = (desiredPosition - arm.getCurrentPosition());
                //double kp = 1;
                //armPower += kp*error;
                //if (error<0) {
                // armPower = -armPower;
                // }
                arm.setPower(armPower);
                while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                runtime.reset();
                racpServo.setPower(racpPower);
                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //go back
                runtime.reset();
                FrontLeft.setPower(0.2);
                BackLeft.setPower(0.2);
                FrontRight.setPower(0.2);
                BackRight.setPower(0.2);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.2)) {
                    telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
            }

        }

    }
}



