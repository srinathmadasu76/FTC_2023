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
    double racpPower = 0.2;
    double armPower = 0.3;
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

        racpServo = hardwareMap.crservo.get("grabberServo");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)  sensorleft;
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)  sensorright;
        waitForStart();
        //Calibration
        FrontLeft.setPower(0.3);
        BackLeft.setPower(-0.3);
        FrontRight.setPower(-0.3);
        BackRight.setPower(0.3);
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
        while (opModeIsActive() && (runtime.seconds() < 1.7)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        FrontLeft.setPower(-0.);
        BackLeft.setPower(-0.);
        FrontRight.setPower(-0.);
        BackRight.setPower(-0.);

        valueleftmeasure = sensorleft.getDistance(DistanceUnit.INCH);
        //while (opModeIsActive() && (runtime.seconds() < 4)) {
           //         telemetry.addData("Sensorright", "Leg 2: %4.1f S Elapsed", valueleftmeasure);
           //    telemetry.update();
          //    }
        if (valueleftmeasure<distancethreshold){
            valueleft = valueleftmeasure;
            FrontLeft.setPower(-0.3);
            BackLeft.setPower(0.3);
            FrontRight.setPower(0.3);
            BackRight.setPower(-0.3);
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
            while (opModeIsActive() && (runtime.seconds() < 1.7)) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(-0.);
            BackRight.setPower(-0.);
        }
        else{

            FrontLeft.setPower(-0.3);
            BackLeft.setPower(0.3);
            FrontRight.setPower(0.3);
            BackRight.setPower(-0.3);
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
            while (opModeIsActive() && (runtime.seconds() < 1.7)) {
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            FrontLeft.setPower(-0.3);
            BackLeft.setPower(-0.3);
            FrontRight.setPower(0.3);
            BackRight.setPower(0.3);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(-0.);
            BackRight.setPower(-0.);
            valueleftmeasure = sensorleft.getDistance(DistanceUnit.INCH);
           // while (opModeIsActive() && (runtime.seconds() < 4)) {
             //   telemetry.addData("Sensormiddle", "Leg 2: %4.1f S Elapsed", valueleftmeasure);
             //   telemetry.update();
          //  }
            if (valueleftmeasure<distancethreshold){
                valuemiddle = valueleftmeasure;
            }

        }
        //while (opModeIsActive() && (runtime.seconds() < 3.8)) {
         //   telemetry.addData("Distance-right1", "Leg 2: %4.1f S Elapsed", valuemiddle);
           // telemetry.update();
       // }

       // FrontLeft.setPower(0.3);
        //BackLeft.setPower(0.3);
        //FrontRight.setPower(-0.3);
        //BackRight.setPower(-0.3);
        //runtime.reset();
       // while (opModeIsActive() && (runtime.seconds() < 0.4)) {
         //   telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
         //   telemetry.update();
      // }
       // FrontLeft.setPower(-0.3);
       // BackLeft.setPower(-0.3);
       // FrontRight.setPower(-0.3);
       // BackRight.setPower(-0.3);
       // runtime.reset();
       // while (opModeIsActive() && (runtime.seconds() < 0.7)) {
        //    telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
         //   telemetry.update();
      //  }

        //valueleft=8;
        if (valueleft > 1 && valueleft < distancethreshold) {
            FrontLeft.setPower(-0.15);
            BackLeft.setPower(-0.15);
            FrontRight.setPower(-0.15);
            BackRight.setPower(-0.15);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }
            FrontLeft.setPower(0.2);
            BackLeft.setPower(0.2);
            FrontRight.setPower(-0.2);
            BackRight.setPower(-0.2);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }

            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(0.);
            BackRight.setPower(0.);
            runtime.reset();
            arm.setPower(armPower);
            turn = motor_ticks_count/4;

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            newTarget = arm.getCurrentPosition() + (int)turn;
            arm.setTargetPosition(newTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && arm.isBusy()) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }
            FrontLeft.setPower(0.15);
            BackLeft.setPower(0.15);
            FrontRight.setPower(0.15);
            BackRight.setPower(0.15);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                telemetry.update();
            }
            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(0.);
            BackRight.setPower(0.);
            sleep(2000);
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
            while (opModeIsActive() && (runtime.seconds() < 0.7)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            FrontLeft.setPower(-0.);
            BackLeft.setPower(-0.);
            FrontRight.setPower(0.);
            BackRight.setPower(0.);
            //double error = (desiredPosition - arm.getCurrentPosition());
            //double kp = 1;
            //armPower += kp*error;
            //if (error<0) {
            // armPower = -armPower;
            // }
            runtime.reset();
            racpServo.setPower(racpPower);
            sleep(1000);

            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            FrontLeft.setPower(-0.5);
            BackLeft.setPower(-0.5);
            FrontRight.setPower(-0.5);
            BackRight.setPower(-0.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.1)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            FrontRight.setPower(0);
            BackRight.setPower(0);




            racpServo.setPower(-racpPower);
            sleep(1000);

            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

        }

        else {

           // FrontLeft.setPower(-0.3);
           // BackLeft.setPower(-0.3);
          //  FrontRight.setPower(0.3);
           // BackRight.setPower(0.3);
           // runtime.reset();
          //  while (opModeIsActive() && (runtime.seconds() < 0.3)) {
           //     telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
           //     telemetry.update();
           // }
           // valueleft = sensorleft.getDistance(DistanceUnit.INCH);
            //telemetry.addData("Distancemiddle", "Leg 2: %4.1f S Elapsed", valueleft);
           // telemetry.update();
            //valueleft = 8;
            if (valuemiddle > 1 && valuemiddle < distancethreshold) {
                FrontLeft.setPower(-0.15);
                BackLeft.setPower(-0.15);
                FrontRight.setPower(-0.15);
                BackRight.setPower(-0.15);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 6)) {
                    telemetry.addData("Distancemiddle", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                FrontLeft.setPower(-0.3);
                BackLeft.setPower(-0.3);
                FrontRight.setPower(0.3);
                BackRight.setPower(0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.01)) {
                    telemetry.addData("Distancemiddle", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                runtime.reset();
                FrontLeft.setPower(-0.);
                BackLeft.setPower(-0.);
                FrontRight.setPower(0.);
                BackRight.setPower(0.);

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
                FrontLeft.setPower(0.15);
                BackLeft.setPower(0.15);
                FrontRight.setPower(0.15);
                BackRight.setPower(0.15);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1.)) {
                    telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                FrontLeft.setPower(-0.);
                BackLeft.setPower(-0.);
                FrontRight.setPower(0.);
                BackRight.setPower(0.);
                sleep(2000);
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

                FrontLeft.setPower(0.4);
                BackLeft.setPower(0.4);
                FrontRight.setPower(0.4);
                BackRight.setPower(0.4);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1.4)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                //double error = (desiredPosition - arm.getCurrentPosition());
                //double kp = 1;
                //armPower += kp*error;
                //if (error<0) {
                // armPower = -armPower;
                // }
                //strafe left
                FrontLeft.setPower(-0.3);
                BackLeft.setPower(-0.3);
                FrontRight.setPower(0.3);
                BackRight.setPower(0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 1)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                runtime.reset();
                racpServo.setPower(racpPower);
                sleep(1000);

                while (opModeIsActive() && (runtime.seconds() < 2)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                //come back
                FrontLeft.setPower(-0.5);
                BackLeft.setPower(-0.5);
                FrontRight.setPower(-0.5);
                BackRight.setPower(-0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.1)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                FrontLeft.setPower(-0.);
                BackLeft.setPower(-0.);
                FrontRight.setPower(0.);
                BackRight.setPower(0.);

                racpServo.setPower(-racpPower);
                sleep(1000);

                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
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
                FrontLeft.setPower(-0.3);
                BackLeft.setPower(-0.3);
                FrontRight.setPower(0.3);
                BackRight.setPower(0.3);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.01)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                FrontLeft.setPower(0.);
                BackLeft.setPower(0.);
                FrontRight.setPower(0.);
                BackRight.setPower(-0.);
                runtime.reset();


                FrontLeft.setPower(-0.15);
                BackLeft.setPower(-0.15);
                FrontRight.setPower(-0.15);
                BackRight.setPower(-0.15);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 2.7)) {
                    telemetry.addData("Distance", "Leg 2: %4.1f S Elapsed", valueleft);
                    telemetry.update();
                }
                FrontLeft.setPower(-0.);
                BackLeft.setPower(-0.);
                FrontRight.setPower(0.);
                BackRight.setPower(0.);

                sleep(2000);
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
                while (opModeIsActive() && (runtime.seconds() < 1.2)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                FrontLeft.setPower(0.);
                BackLeft.setPower(-0.);
                FrontRight.setPower(-0.);
                BackRight.setPower(0.);


                //come back
                FrontLeft.setPower(-0.5);
                BackLeft.setPower(-0.5);
                FrontRight.setPower(-0.5);
                BackRight.setPower(-0.5);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.2)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                FrontLeft.setPower(0.);
                BackLeft.setPower(-0.);
                FrontRight.setPower(-0.);
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

                //double error = (desiredPosition - arm.getCurrentPosition());
                //double kp = 1;
                //armPower += kp*error;
                //if (error<0) {
                // armPower = -armPower;

                runtime.reset();
                racpServo.setPower(racpPower);
                sleep(1000);

                while (opModeIsActive() && (runtime.seconds() < 2)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                racpServo.setPower(-racpPower);
                sleep(1000);

                while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
            }

        }

    }
}



