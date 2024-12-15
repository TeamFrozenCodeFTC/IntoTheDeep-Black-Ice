package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

// A parent class to all operation modes. Contains the Robot's Hardware but also LinearOpMode.
public abstract class Robot extends LinearOpMode {
    public DcMotor frontLeftWheel;
    public DcMotor backLeftWheel;
    public DcMotor frontRightWheel;
    public DcMotor backRightWheel;

    public DcMotor viperSlideMotor;

    public DcMotor intakeExtender;
    public CRServo sweeper;
    public Servo sweeperRotator;

    public Gyro2 gyro;

    public Servo dumperServo;
    public Servo clawLeft;
    public Servo clawRight;

    public ViperSlide viperSlide;
    public Intake intake;

    public TouchSensor touchLeft;
    public TouchSensor touchRight;

    private void autoBrake(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void unbrakeMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void reverse(DcMotor motor) {
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void resetTicks(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPositionMode(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void drift(double seconds) {
        unbrakeMotor(frontRightWheel);
        unbrakeMotor(frontLeftWheel);
        unbrakeMotor(backRightWheel);
        unbrakeMotor(backLeftWheel);

        sleep((int) seconds* 1000L);

        autoBrake(frontRightWheel);
        autoBrake(frontLeftWheel);
        autoBrake(backRightWheel);
        autoBrake(backLeftWheel);
    }

    public void initWheels() {
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeft");
        autoBrake(frontLeftWheel);
        reverse(frontLeftWheel);

        backLeftWheel = hardwareMap.get(DcMotor.class, "backLeft");
        autoBrake(backLeftWheel);

        frontRightWheel = hardwareMap.get(DcMotor.class, "frontRight");
        autoBrake(frontRightWheel);
        reverse(frontRightWheel);

        backRightWheel = hardwareMap.get(DcMotor.class, "backRight");
        autoBrake(backRightWheel);
    }

    public void initHardware() {
        initWheels();

        intakeExtender = hardwareMap.get(DcMotor.class, "intakeMotor");
        autoBrake(intakeExtender);
        reverse(intakeExtender);

        viperSlideMotor = hardwareMap.get(DcMotor.class, "linearSlide");
        reverse(viperSlideMotor);
        runToPositionMode(viperSlideMotor);

        sweeper = hardwareMap.get(CRServo.class, "sweeper");
        sweeper.setDirection(DcMotorSimple.Direction.REVERSE);

        sweeperRotator = hardwareMap.get(Servo.class, "sweeperRotator");

        dumperServo = hardwareMap.get(Servo.class, "dumperServo");

        touchLeft = hardwareMap.get(TouchSensor.class, "touchLeft");
        touchRight = hardwareMap.get(TouchSensor.class, "touchRight");

        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        gyro = new Gyro2(imu, this);
        gyro.startGyro();
        gyro.reset();
    }

    public void initRobot() {
        initHardware();

        viperSlide = new ViperSlide(this);
        intake = new Intake(this);

        intake.armIn();
        viperSlide.bucketDown();
        viperSlide.clawOut();
    }
}
