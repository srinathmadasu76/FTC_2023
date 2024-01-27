package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Autonomous(name="Auton_DropPixel_BlueAllianceRear", group="Robot")
//@Disabled
public class Auton_DropPixel_BlueAllianceRear extends LinearOpMode {
    private DistanceSensor sensorleft;
    private DistanceSensor sensorright;
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;
    //DcMotor Intake = null;
    DcMotor arm= null;
    CRServo bucketServo;
    private ElapsedTime runtime = new ElapsedTime();
    double bucketPower = 0.2;
    double armPower = 0.3;
    double armPowerRetract = 0.2;
    double motor_ticks_count = 728;
    double turn;
    double desiredPosition = 10;
    int newTarget;

    double valueleft = 300;
    double valuemiddle = 300;
    double valueleftmeasure;
    double distancethreshold = 24;
    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorleft = hardwareMap.get(DistanceSensor.class, "sensor_left");
        //sensorright = hardwareMap.get(DistanceSensor.class, "sensor_right");
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

        bucketServo = hardwareMap.crservo.get("grabberServo");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)  sensorleft;
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)  sensorright;
        waitForStart();

        FrontLeft.setPower(-0.3);
        BackLeft.setPower(0.3);
        FrontRight.setPower(0.3);
        BackRight.setPower(-0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(-0.2);
        BackLeft.setPower(-0.2);
        FrontRight.setPower(-0.2);
        BackRight.setPower(-0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(-0.);
        BackLeft.setPower(-0.);
        FrontRight.setPower(-0.);
        BackRight.setPower(-0.);

        valueleftmeasure = sensorleft.getDistance(DistanceUnit.INCH);

        if (valueleftmeasure<distancethreshold){
            valueleft = valueleftmeasure;
        }
        else{

            FrontLeft.setPower(0.3);
            BackLeft.setPower(-0.3);
            FrontRight.setPower(-0.3);
            BackRight.setPower(0.3);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.3)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            FrontLeft.setPower(0.2);
            BackLeft.setPower(0.2);
            FrontRight.setPower(0.2);
            BackRight.setPower(0.2);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.7)) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(-0.);
            BackRight.setPower(-0.);
            valueleftmeasure = sensorleft.getDistance(DistanceUnit.INCH);

            if (valueleftmeasure<distancethreshold){
                valuemiddle = valueleftmeasure;
            }

        }
      //  while (opModeIsActive() && (runtime.seconds() < 3.8)) {
        //    telemetry.addData("Distance-right1", "Leg 2: %4.1f S Elapsed", valuemiddle);
         //   telemetry.update();
      //  }
        FrontLeft.setPower(-0.3);
        BackLeft.setPower(-0.3);
        FrontRight.setPower(0.3);
        BackRight.setPower(0.3);
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
        if (valueleft > 1 && valueleft < distancethreshold) {
            FrontLeft.setPower(-0.15);
            BackLeft.setPower(-0.15);
            FrontRight.setPower(-0.15);
            BackRight.setPower(-0.15);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 4)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }
            FrontLeft.setPower(-0.2);
            BackLeft.setPower(-0.2);
            FrontRight.setPower(0.2);
            BackRight.setPower(0.2);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.01)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }
            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(0.);
            BackRight.setPower(0.);
            runtime.reset();

            turn = motor_ticks_count/4;

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newTarget = arm.getCurrentPosition() + (int)turn;
            arm.setTargetPosition(newTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            while (opModeIsActive() && arm.isBusy()) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }
            //arm.setPower(0.);
            FrontLeft.setPower(-0.2);
            BackLeft.setPower(-0.2);
            FrontRight.setPower(-0.2);
            BackRight.setPower(-0.2);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.2)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }

            //spin clockwise
            runtime.reset();
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(-0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            //Go to Park
            FrontLeft.setPower(0.5);
            BackLeft.setPower(0.5);
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.9)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            //double error = (desiredPosition - arm.getCurrentPosition());
            //double kp = 1;
            //armPower += kp*error;
            //if (error<0) {
            // armPower = -armPower;
            // }
            FrontLeft.setPower(-0.3);
            BackLeft.setPower(-0.3);
            FrontRight.setPower(0.3);
            BackRight.setPower(0.3);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(0.);
            BackRight.setPower(0.);
            runtime.reset();
            bucketServo.setPower(bucketPower);
            sleep(1000);

            while (opModeIsActive() && (runtime.seconds() < 2)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            bucketServo.setPower(-bucketPower);
            sleep(1000);

            while (opModeIsActive() && (runtime.seconds() < 10)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
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
            valueleft = sensorleft.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distancemiddle", "Leg 2: %4.1f S Elapsed", valueleft);
            telemetry.update();
            //valueleft=26;
            if (valuemiddle > 1 && valuemiddle < distancethreshold) {
                FrontLeft.setPower(-0.15);
                BackLeft.setPower(-0.15);
                FrontRight.setPower(-0.15);
                BackRight.setPower(-0.15);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3)) {
                    telemetry.addData("Distancemiddle", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                FrontLeft.setPower(0.3);
                BackLeft.setPower(0.3);
                FrontRight.setPower(-0.3);
                BackRight.setPower(-0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.1)) {
                    telemetry.addData("Distancemiddle", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                runtime.reset();


                turn = motor_ticks_count/4;
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                newTarget = arm.getCurrentPosition() + (int)turn;
                arm.setTargetPosition(newTarget);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);
                while (opModeIsActive() && arm.isBusy()) {
                    telemetry.addData("Distancemiddle", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                //arm.setPower(0.);
                FrontLeft.setPower(-0.2);
                BackLeft.setPower(-0.2);
                FrontRight.setPower(-0.2);
                BackRight.setPower(-0.2);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1.2)) {
                    telemetry.addData("Distancemiddle", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                //spin clockwise
                runtime.reset();
                FrontLeft.setPower(-0.5);
                BackLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackRight.setPower(-0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //Go to Park
                FrontLeft.setPower(0.5);
                BackLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackRight.setPower(0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3.9)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                //double error = (desiredPosition - arm.getCurrentPosition());
                //double kp = 1;
                //armPower += kp*error;
                //if (error<0) {
                // armPower = -armPower;
                // }
                FrontLeft.setPower(-0.3);
                BackLeft.setPower(-0.3);
                FrontRight.setPower(0.3);
                BackRight.setPower(0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                FrontLeft.setPower(-0.);
                BackLeft.setPower(-0.);
                FrontRight.setPower(0.);
                BackRight.setPower(0.);
                runtime.reset();
                bucketServo.setPower(bucketPower);
                sleep(1000);

                while (opModeIsActive() && (runtime.seconds() < 2)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                bucketServo.setPower(-bucketPower);
                sleep(1000);

                while (opModeIsActive() && (runtime.seconds() < 10)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
            }
            else {
                runtime.reset();
                FrontLeft.setPower(-0.15);
                BackLeft.setPower(-0.15);
                FrontRight.setPower(-0.15);
                BackRight.setPower(-0.15);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.01)) {
                    telemetry.addData("Distanceright", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }

                FrontLeft.setPower(0.3);
                BackLeft.setPower(0.3);
                FrontRight.setPower(-0.3);
                BackRight.setPower(-0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.01)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                FrontLeft.setPower(0.);
                BackLeft.setPower(0.);
                FrontRight.setPower(0.);
                BackRight.setPower(-0.);


                FrontLeft.setPower(-0.2);
                BackLeft.setPower(-0.2);
                FrontRight.setPower(-0.2);
                BackRight.setPower(-0.2);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1.8)) {
                    telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                //spin clockwise
                runtime.reset();
                FrontLeft.setPower(-0.5);
                BackLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackRight.setPower(-0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.9)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //Go to Park
                FrontLeft.setPower(0.5);
                BackLeft.setPower(0.5);
                FrontRight.setPower(0.5);
                BackRight.setPower(0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3.9)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                //double error = (desiredPosition - arm.getCurrentPosition());
                //double kp = 1;
                //armPower += kp*error;
                //if (error<0) {
                // armPower = -armPower;
                // }
                FrontLeft.setPower(-0.3);
                BackLeft.setPower(-0.3);
                FrontRight.setPower(0.3);
                BackRight.setPower(0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                FrontLeft.setPower(-0.);
                BackLeft.setPower(-0.);
                FrontRight.setPower(0.);
                BackRight.setPower(0.);
                runtime.reset();
                turn = motor_ticks_count/4;

                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                newTarget = arm.getCurrentPosition() + (int)turn;
                arm.setTargetPosition(newTarget);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armPower);
                while (opModeIsActive() && arm.isBusy()) {
                    telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                runtime.reset();
                bucketServo.setPower(bucketPower);
                sleep(1000);

                while (opModeIsActive() && (runtime.seconds() < 2)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                bucketServo.setPower(-bucketPower);
                sleep(1000);

                while (opModeIsActive() && (runtime.seconds() < 7)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
            }

        }

    }
}

