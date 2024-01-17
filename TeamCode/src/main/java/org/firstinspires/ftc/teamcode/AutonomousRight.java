package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autonomous Right India", group = "BTJ #13452")
// @Disabled
public class AutonomousRight extends LinearOpMode {
    BNO055IMU imu;
    public ElapsedTime runTime = new ElapsedTime();
    Orientation lastAngles = new Orientation();
    double globalAngle;

    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    Servo turnPump;
    Servo PumpDirectionChangerRight;
    Servo PumpDirectionChangerLeft;

    CRServo pumpRight;
    CRServo pumpLeft;

    int stackHeightIndex = 0;

    DcMotor elevatorRight;
    DcMotor elevatorLeft;

    int[] coneStackHeights = new int[]{446,337,234,125,0};

    ColorSensor sleeveSensor;
    DistanceSensor distanceRight;
    DistanceSensor distanceLeft;
    DistanceSensor findPole;
    DistanceSensor findJunction;
    TouchSensor stopP;
    
    String sleeveColor = "none";

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        motorFrontLeft = hardwareMap.dcMotor.get("UL");
        motorBackLeft = hardwareMap.dcMotor.get("DL");
        motorFrontRight = hardwareMap.dcMotor.get("UR");
        motorBackRight = hardwareMap.dcMotor.get("DR");

        elevatorRight = hardwareMap.dcMotor.get("ER");
        elevatorLeft = hardwareMap.dcMotor.get("EL");

        stopP = hardwareMap.touchSensor.get("stopPump");

        pumpRight = hardwareMap.crservo.get("pr");
        pumpLeft = hardwareMap.crservo.get("pl");

        distanceRight = hardwareMap.get(DistanceSensor.class, "DistanceRight");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "DistanceLeft");
        findPole = hardwareMap.get(DistanceSensor.class, "findPoleBack");
        findJunction = hardwareMap.get(DistanceSensor.class, "findJunction");


        sleeveSensor = hardwareMap.get(ColorSensor.class, "colorSensorLeft");

        turnPump = hardwareMap.servo.get("rotatePump");
        PumpDirectionChangerRight = hardwareMap.servo.get("sideLeft");
        PumpDirectionChangerLeft = hardwareMap.servo.get("sideRight");

        // set direction of needed motors to reverse
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set encoders to 0
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackRight.setTargetPosition(0);
        motorBackLeft.setTargetPosition(0);
        motorFrontRight.setTargetPosition(0);
        motorFrontLeft.setTargetPosition(0);

        runTime.reset();


        // set encoders
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turnPump.setPosition(0.99);
        PumpDirectionChangerLeft.setPosition(1);
        PumpDirectionChangerRight.setPosition(0);

        waitForStart();

        setElevatorHeight(200);
        driveByEncoder(0.6, -90, 52, 0);
        sleep(200);
        ScanForColor();
        setElevatorHeight(2150);
        driveByEncoder(0.6, -90, 60, 0);
        sleep(200);
        driveByEncoder(-1,0,8,0);
        driveToJunction(-0.2,0,0);
        driveByEncoder(-0.2,0,2,0);
        pumpRight.setPower(1);
        pumpLeft.setPower(-1);
        sleep(400);
        driveByEncoder(0.6, 0, 8, 0);
        new Thread(new Runnable() {
            @Override
            public void run() {
                turnPump.setPosition(0.3);
                sleep(500);
                PumpDirectionChangerLeft.setPosition(0);
                PumpDirectionChangerRight.setPosition(1);
                sleep(200);
                setElevatorHeight(coneStackHeights[stackHeightIndex]);
            }
        }).start();
        sleep(200);
        driveByEncoder(0.6, -90, 30, 0);
        for(int i = 0; i < 4; i++){
            sleep(200);
            stackHeightIndex++;
            driveByEncoder(1,0,50,0);
            sleep(200);
            driveToWall(0.3,0,0);
            sleep(100);
            pumpRight.setPower(0);
            pumpLeft.setPower(0);
            sleep(100);
            setElevatorHeight(2150);
            sleep(250);
            new Thread(new Runnable(){
                @Override
                public void run() {
                    turnPump.setPosition(0.99);
                    sleep(600);
                    PumpDirectionChangerLeft.setPosition(1);
                    PumpDirectionChangerRight.setPosition(0);
                }
            }).start();
            driveByEncoder(-0.8,0,51,0);
            sleep(200);
            driveByEncoder(0.8,90,36,0);
            driveUntilJunction(0.25,90,0);
            driveByEncoder(0.25,90,6,0);
            sleep(100);
            driveByEncoder(-1,0,8,0);
            driveToJunction(-0.2,0,0);
            driveByEncoder(-0.2,0,1,0);
            pumpRight.setPower(1);
            pumpLeft.setPower(-1);
            if(i == 3){
                sleep(300);
            }
            sleep(400);
            new Thread(new Runnable(){
                @Override
                public void run() {
                    sleep(400);
                    setElevatorHeight(coneStackHeights[stackHeightIndex]);
                    turnPump.setPosition(0.3);
                    sleep(300);
                    PumpDirectionChangerLeft.setPosition(0);
                    PumpDirectionChangerRight.setPosition(1);
                }
            }).start();
            driveByEncoder(0.6,0,8,0);
            // driveByEncoder(0.3,0,19,0);
            sleep(100);
            driveByEncoder(0.6,-90,31,0);
        }

        if(sleeveColor == "red"){
            driveByEncoder(-1,0,50,0);
        }
        if(sleeveColor == "blue"){
            driveByEncoder(1,0,50,0);
        }
        stopRobot();
        sleep(3000);
    }


    public void setElevatorHeight(int ticks){
        elevatorRight.setTargetPosition(ticks);
        elevatorLeft.setTargetPosition(ticks);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setPower(1);
        elevatorLeft.setPower(1);
    }

    private void stopRobot() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return -globalAngle;
    }

    public void driveByEncoder(double power, double angleDegrees, double CM, int angleAheadDegrees) {
        // 9.6 CM diameter
        // 9.6 * PI = perimeter
        // perimeter = 30.16
        // encoder for one round = 384.5
        // (30.16 / 384.5) * encoder = CM
        // 0.0784 * encoder = CM
        // encoder = CM / 0.0561
        double encoder = CM / 0.0784;

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();
        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getAngle()) * 0.01;
        //Denominator is the largest motor power (absolute value) or 1
        //This ensures all the powers maintain the same ratio, but only when
        //at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        double frontRightEncoder = encoder;
        double backRightEncoder = encoder;


        while ((Math.abs(motorFrontRight.getCurrentPosition()) < frontRightEncoder || Math.abs(motorBackRight.getCurrentPosition()) < backRightEncoder)&&opModeIsActive()) {
            rx = -(angleAheadDegrees - getAngle()) * 0.01;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if ((motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1)&&  opModeIsActive()) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    /*
    int Red = 1300;
    int Green = 3500;
    int Blue = 2500;
    public void ScanForColor() {
        if (Green < 500) {
            if (sleeveSensor.red() > sleeveSensor.green() && sleeveSensor.red() > sleeveSensor.blue())
                sleeveColor = "red";
            if (sleeveSensor.blue() > sleeveSensor.green() && sleeveSensor.blue() > sleeveSensor.red())
                sleeveColor = "blue";
            if (sleeveSensor.green() > sleeveSensor.red() && sleeveSensor.green() > sleeveSensor.blue())
                sleeveColor = "green";
        }
        if (sleeveSensor.red() > Red) sleeveColor = "red";
        if (sleeveSensor.blue() > Blue) sleeveColor = "blue";
        if (sleeveSensor.green() > Green) sleeveColor = "green";
        if (sleeveColor == "none") {
            Red -= 100;
            Blue -= 200;
            Green -= 300;
            ScanForColor();
        }
    }
    */
    public void ScanForColor(){
        if(sleeveSensor.red() > sleeveSensor.green() && sleeveSensor.red() > sleeveSensor.blue())
            sleeveColor = "red";
        if(sleeveSensor.blue() > sleeveSensor.green() && sleeveSensor.blue() > sleeveSensor.red())
            sleeveColor = "blue";
        if(sleeveSensor.green() > sleeveSensor.red() && sleeveSensor.green() > sleeveSensor.blue())
            sleeveColor = "green";
        if(sleeveColor == "none")
            ScanForColor();

        telemetry.addData("red", sleeveSensor.red());
        telemetry.addData("green", sleeveSensor.green());
        telemetry.addData("blue", sleeveSensor.blue());
        telemetry.addData("color",sleeveColor);
        telemetry.update();
    }
    public void driveToWall(double power, double angleDegrees, int angleAheadDegrees) {

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getAngle()) * 0.01;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        while (verifyDistance() && opModeIsActive()) {
            rx = -(angleAheadDegrees - getAngle()) * 0.01;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public boolean verifyDistance(){
        int counter = 0;
        boolean current = false;
        while(counter < 3 &&opModeIsActive()){
            current = distanceRight.getDistance(DistanceUnit.CM) > 3 || distanceLeft.getDistance(DistanceUnit.CM) > 3;
            if(current == distanceRight.getDistance(DistanceUnit.CM) > 3 || distanceLeft.getDistance(DistanceUnit.CM) > 3){
                counter++;
            }
            else{
                counter = 0;
            }
        }
        return current;
    }

    public void driveToJunction(double power, double angleDegrees, int angleAheadDegrees) {
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getAngle()) * 0.01;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        while (verifyFindPole() && opModeIsActive()) {
            rx = -(angleAheadDegrees - getAngle()) * 0.01;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
    public boolean verifyFindPole(){
        int counter = 0;
        boolean current = false;
        while(counter < 3 && opModeIsActive()){
            current = findPole.getDistance(DistanceUnit.CM) > 34;
            if(current == findPole.getDistance(DistanceUnit.CM) > 34){
                counter++;
            }
            else{
                counter = 0;
            }
        }
        return current;
    }

    public void driveUntilJunction(double power, double angleDegrees, int angleAheadDegrees) {
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        double y = -(power * Math.sin(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double x = -(power * Math.cos(Math.toRadians(-angleDegrees) + Math.PI / 2));
        double rx = -(angleAheadDegrees - getAngle()) * 0.01;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        while (verifyJunction() && opModeIsActive()) {
            rx = -(angleAheadDegrees - getAngle()) * 0.01;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            if (motorFrontRight.getCurrentPosition() == 0 && motorBackRight.getCurrentPosition() == 0 && runTime.seconds() > 1) {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    public boolean verifyJunction(){
        int counter = 0;
        boolean current = false;
        while(counter < 1 && opModeIsActive()){
            current = findJunction.getDistance(DistanceUnit.CM) > 50;
            if(current == findJunction.getDistance(DistanceUnit.CM) > 50){
                counter++;
            }
            else{
                counter = 0;
            }
        }
        return current;
    }
}