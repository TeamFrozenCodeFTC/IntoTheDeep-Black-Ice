package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.autonomous.custom.GoBildaPinpointDriver;

// A parent class to all operation modes. Contains the Robot's Hardware but also LinearOpMode.
public abstract class OldRobot extends LinearOpMode {
    public DcMotor frontLeftWheel;
    public DcMotor backLeftWheel;
    public DcMotor frontRightWheel;
    public DcMotor backRightWheel;

    public DcMotor viperSlideMotor;

    public DcMotor intakeExtender;
    public CRServo sweeper;
    public Servo sweeperRotator;

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

    public GoBildaPinpointDriver odometry;

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

        odometry = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odometry.setOffsets(-36, 0);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();
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
