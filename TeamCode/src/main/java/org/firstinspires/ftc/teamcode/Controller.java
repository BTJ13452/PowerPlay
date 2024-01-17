package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Controller", group = "BTJ #13452")
// @Disabled
public class Controller extends LinearOpMode {

    //Controller ***************************************************************

    //Vibration
    Gamepad.RumbleEffect EndGameVibration;
    Gamepad.RumbleEffect lastTenSecondsVibration;
    Gamepad.RumbleEffect readyToReleaseVibration;
    Gamepad.RumbleEffect closeToJunctionVibration;

    //Controller ***************************************************************

    //Drive variables***********************************************************
    double power;
    double driverAngle;
    double rotation;
    double targetAngle;

    //Drive variables***********************************************************

    //Mechanics***************************************************************

    //Wheels
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    //Elevators
    DcMotor elevatorRight;
    DcMotor elevatorLeft;
    //Pumping system
    CRServo pumpRight;
    CRServo pumpLeft; 
    Servo turnPump; 
    Servo pumpDirectionChangerRight;
    Servo pumpDirectionChangerLeft;
    //Sensors
    TouchSensor stopP;

    DistanceSensor findPole;
    DistanceSensor findPoleFront;
    DistanceSensor findPoleBack;

    //Mechanics***************************************************************

    //Program variables*********************************************************

    // IsPressed
    boolean xIsPressed = false;
    boolean stopPIsPressed = false;
    boolean BRIsPressed = false;
    boolean BLIsPressed = false;
    boolean aIsPressed = false;
    boolean bIsPressed = false;
    boolean yIsPressed = false;
    boolean rightTriggerWasPressed = false;
    boolean leftTriggerWasPressed = false;
    boolean rightStickButtonIsPressed = false;

    //algorithmic
    int minHeightForJunctionIdentification = 700;
    int minHeightForConeIdentification = 100;
    int maxElevatorHeight = 3000;
    int minElevatorHeight = 1;
    int targetStackHeight = 0;
    int targetHeight = 0;
    int rou = 0;
    final int[] coneStackHeight = new int[] {446,337,234,125};
    final int[] junctionsLevels = new int[] {0,1300,2100,2900};

    //Program variables*********************************************************

    @Override
    public void runOpMode() throws InterruptedException {

        //Configure*************************************************************

        //Find on hardware
        motorFrontLeft = hardwareMap.dcMotor.get("UL");
        motorBackLeft = hardwareMap.dcMotor.get("DL");
        motorFrontRight = hardwareMap.dcMotor.get("UR");
        motorBackRight = hardwareMap.dcMotor.get("DR");

        elevatorRight = hardwareMap.dcMotor.get("ER");
        elevatorLeft = hardwareMap.dcMotor.get("EL");

        turnPump = hardwareMap.servo.get("rotatePump");
        pumpDirectionChangerRight = hardwareMap.servo.get("sideRight");
        pumpDirectionChangerLeft = hardwareMap.servo.get("sideLeft");

        pumpRight = hardwareMap.crservo.get("pr");
        pumpLeft = hardwareMap.crservo.get("pl");

        stopP = hardwareMap.touchSensor.get("stopPump");

        findPoleFront = hardwareMap.get(DistanceSensor.class, "findPoleFront");
        findPoleBack = hardwareMap.get(DistanceSensor.class, "findPoleBack");
        findPole = findPoleBack;

        //Settings
        findPole.resetDeviceConfigurationForOpMode();

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorRight.setTargetPosition(0);
        elevatorLeft.setTargetPosition(0);

        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Configure*************************************************************

        //Controller****************************************************************

        EndGameVibration = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 250)
                .addStep(0.0, 0.0, 100)
                .addStep(1.0, 1.0, 250)
                .build();

        lastTenSecondsVibration = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 1000)
                .addStep(0.0, 0.0, 1000)
                .addStep(0.7, 0.7, 1000)
                .addStep(0.0, 0.0, 1000)
                .addStep(1.0, 1.0, 1000)
                .build();

        readyToReleaseVibration = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0,1.0,250)
                .build();

        //Controller************************************************************

        //INIT movements********************************************************

        turnPump.setPosition(0.315);
        pumpDirectionChangerLeft.setPosition(1);
        pumpDirectionChangerRight.setPosition(0);

        //INIT movements********************************************************

        waitForStart();//*******************************************************

        //GameTimer*************************************************************

        new Thread(new Runnable() {
            @Override
            public void run() {
                sleep(90000);
                gamepad1.runRumbleEffect(EndGameVibration);
                sleep(25000);
                telemetry.addData("hi","");
                telemetry.update();
                gamepad1.runRumbleEffect(lastTenSecondsVibration);
            }
        }).start();

        //GameTimer*************************************************************

        while (opModeIsActive()) {

            if(gamepad1.a && !aIsPressed ){
                aIsPressed = true;
                rou++;
                telemetry.addData("hi",rou);
                telemetry.update();
                // stopPIsPressed = false;
                if(pumpRight.getPower() != 1){
                    pumpRight.setPower(1);
                    pumpLeft.setPower(-1);
                } else{
                    pumpRight.setPower(0);
                    pumpLeft.setPower(0);
                }
            }else if(!gamepad1.a && aIsPressed){
                aIsPressed = false;
            }

            if(gamepad1.b && !bIsPressed){
                bIsPressed = true;
                // stopPIsPressed = false;
                if(pumpRight.getPower() != -1){
                    pumpRight.setPower(-1);
                    pumpLeft.setPower(1);
                }
                else{
                    pumpRight.setPower(0);
                    pumpLeft.setPower(0);
                }
            }else if(!gamepad1.b && bIsPressed){
                bIsPressed = false;
            }


            if(gamepad1.right_stick_button &&!rightStickButtonIsPressed){
                rightStickButtonIsPressed = true;

                elevatorRight.setTargetPosition(coneStackHeight[targetStackHeight]);
                elevatorLeft.setTargetPosition(coneStackHeight[targetStackHeight]);
                elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorRight.setPower(1);
                elevatorLeft.setPower(1);

                targetStackHeight++;
                if (targetStackHeight > 3) targetStackHeight = 3;
            } else if (!gamepad1.right_stick_button) rightStickButtonIsPressed = false;

            if(gamepad1.y && !yIsPressed) {
                yIsPressed = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        if (pumpDirectionChangerLeft.getPosition() == 0){
                            findPole = findPoleFront;
                            turnPump.setPosition(0.315);
                            sleep(400);
                            pumpDirectionChangerLeft.setPosition(1);
                            pumpDirectionChangerRight.setPosition(0);
                        } else if (pumpDirectionChangerRight.getPosition() == 0){
                            findPole = findPoleBack;
                            turnPump.setPosition(0.97);
                            sleep(400);
                            pumpDirectionChangerLeft.setPosition(0);
                            pumpDirectionChangerRight.setPosition(1);
                        }
                    }
                }).start();
            } else if (!gamepad1.y) {
                yIsPressed = false;
            }

            if(gamepad1.left_stick_button && !rightStickButtonIsPressed){
                rightStickButtonIsPressed = true;
                elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if(!gamepad1.right_stick_button) rightStickButtonIsPressed = false;

            if(gamepad1.x && !xIsPressed){
                targetStackHeight = 0;
                xIsPressed = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        if (pumpDirectionChangerLeft.getPosition() == 0){
                            findPole = findPoleFront;
                            targetHeight = 0;
                            setElevatorHeight(0);
                            turnPump.setPosition(0.315);
                            sleep(350);
                            pumpDirectionChangerLeft.setPosition(1);
                            pumpDirectionChangerRight.setPosition(0);
                        } else if (pumpDirectionChangerLeft.getPosition() == 1){
                            findPole = findPoleBack;
                            targetHeight = 3;
                            setElevatorHeight(3);
                            turnPump.setPosition(0.97);
                            sleep(350);
                            pumpDirectionChangerLeft.setPosition(0);
                            pumpDirectionChangerRight.setPosition(1);
                        }
                    }
                }).start();
            }

            if(!gamepad1.x && xIsPressed){
                xIsPressed = false;
            }


            if(gamepad1.dpad_left){
                turnPump.setPosition(0.97);
            }
            if(gamepad1.dpad_right){
                turnPump.setPosition(0.315);
            }
            if (gamepad1.dpad_up){
                pumpDirectionChangerLeft.setPosition(1);
                pumpDirectionChangerRight.setPosition(0);
                findPole = findPoleFront;
            }
            if (gamepad1.dpad_down){
                pumpDirectionChangerLeft.setPosition(0);
                pumpDirectionChangerRight.setPosition(1);
                findPole = findPoleBack;
            }

            if (findPole.getDistance(DistanceUnit.CM) < 33 && findPole.getDistance(DistanceUnit.CM) > 22 && elevatorRight.getCurrentPosition() > minHeightForJunctionIdentification){
                gamepad1.runRumbleEffect(readyToReleaseVibration);
            } else if (findPole.getDistance(DistanceUnit.CM) < 40  && findPole.getDistance(DistanceUnit.CM) > 33  && elevatorRight.getCurrentPosition() > minHeightForJunctionIdentification) {
                closeToJunctionVibration = new Gamepad.RumbleEffect.Builder()
                        .addStep(1-((40 - findPole.getDistance(DistanceUnit.CM))/14),0.0,100)
                        .build();
                gamepad1.runRumbleEffect(closeToJunctionVibration);
            }  else if (findPole.getDistance(DistanceUnit.CM) < 22 && findPole.getDistance(DistanceUnit.CM) > 15 && elevatorRight.getCurrentPosition() > minHeightForJunctionIdentification){
                closeToJunctionVibration = new Gamepad.RumbleEffect.Builder()
                        .addStep(0.0,0.5 + ((findPole.getDistance(DistanceUnit.CM)-15)/7),100)
                        .build();
                gamepad1.runRumbleEffect(closeToJunctionVibration);
            }

            if(stopP.isPressed() && !stopPIsPressed && elevatorLeft.getCurrentPosition() < minHeightForConeIdentification){
                stopPIsPressed = true;

                pumpLeft.setPower(0);
                pumpRight.setPower(0);

                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        sleep(250);

                        if(elevatorRight.getTargetPosition() < minHeightForConeIdentification){
                            elevatorRight.setTargetPosition(150);
                            elevatorLeft.setTargetPosition(150);

                            elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            elevatorRight.setPower(1);
                            elevatorLeft.setPower(1);
                        }
                    }
                }).start();

            } else if(!stopP.isPressed() && stopPIsPressed){
                stopPIsPressed = false;
            }

            if(gamepad1.right_bumper && !BRIsPressed){
                targetStackHeight = 0;
                BRIsPressed = true;
                targetHeight++;
                if (targetHeight > 3) targetHeight = 3;
                setElevatorHeight(targetHeight);
            } else if(!gamepad1.right_bumper && BRIsPressed){
                BRIsPressed = false;
            }

            if(gamepad1.left_bumper && !BLIsPressed){
                targetStackHeight = 0;
                BLIsPressed = true;
                targetHeight--;
                if (targetHeight <= 0) targetHeight = 0;
                setElevatorHeight(targetHeight);
            } else if(!gamepad1.left_bumper && BLIsPressed){
                BLIsPressed = false;
            }

            if(gamepad1.right_trigger > 0 && elevatorRight.getCurrentPosition()<maxElevatorHeight){
                targetStackHeight = 0;
                rightTriggerWasPressed = true;
                elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorRight.setPower(gamepad1.right_trigger);
                elevatorLeft.setPower(gamepad1.right_trigger);
            }else if(rightTriggerWasPressed){
                rightTriggerWasPressed = false;
                elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorRight.setTargetPosition(elevatorRight.getCurrentPosition());
                elevatorLeft.setTargetPosition(elevatorLeft.getCurrentPosition());
                elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorRight.setPower(1);
                elevatorLeft.setPower(1);

                if(elevatorRight.getCurrentPosition()>junctionsLevels[2]+10)
                    targetHeight = 3;
                else if (elevatorRight.getCurrentPosition()>junctionsLevels[1]+10)
                    targetHeight = 2;
                else if (elevatorRight.getCurrentPosition()>junctionsLevels[0]+10)
                    targetHeight = 1;
                else targetHeight = 0;
            }

            if(gamepad1.left_trigger > 0){
                targetStackHeight = 0;
                leftTriggerWasPressed = true;
                elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorRight.setPower(-gamepad1.left_trigger);
                elevatorLeft.setPower(-gamepad1.left_trigger);
            } else if(leftTriggerWasPressed){
                leftTriggerWasPressed = false;
                elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorRight.setTargetPosition(elevatorRight.getCurrentPosition());
                elevatorLeft.setTargetPosition(elevatorLeft.getCurrentPosition());
                elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorRight.setPower(1);
                elevatorLeft.setPower(1);

                if(elevatorRight.getCurrentPosition()>junctionsLevels[2]+10)
                    targetHeight = 3;
                else if (elevatorRight.getCurrentPosition()>junctionsLevels[1]+10)
                    targetHeight = 2;
                else if (elevatorRight.getCurrentPosition()>junctionsLevels[0]+10)
                    targetHeight = 1;
                else targetHeight = 0;
            }

            //Drive actions*********************************************************

            power = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));

            driverAngle = Math.atan(-gamepad1.left_stick_y / gamepad1.left_stick_x);

            rotation = gamepad1.right_stick_x;

            if (Double.isNaN(driverAngle))
                driverAngle = 0;

            if (gamepad1.left_stick_x < 0)
                driverAngle += Math.PI;

            targetAngle = driverAngle; //- (-Math.toRadians(getAngle()));

            drive(power, targetAngle, rotation);

            //Drive actions*****************************************************
        }
    }

    public void drive(double power, double angle, double rotation) {

        double y = -(power * Math.sin(angle));
        double x = -(power * Math.cos(angle));
        double rx = -rotation;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    public void setElevatorHeight(int targetHeight) {
        elevatorRight.setTargetPosition(junctionsLevels[targetHeight]);
        elevatorLeft.setTargetPosition(junctionsLevels[targetHeight]);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(1);
        elevatorLeft.setPower(1);
    }
}