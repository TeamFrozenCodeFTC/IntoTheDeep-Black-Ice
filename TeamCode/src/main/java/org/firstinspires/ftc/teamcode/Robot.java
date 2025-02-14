package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.MovementBuilder;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

// A parent class to all operation modes. Contains the Robot's Hardware but also LinearOpMode.
public abstract class Robot extends LinearOpMode {
    // 7 Motors
    public DcMotor frontLeftWheel;
    public DcMotor backLeftWheel;
    public DcMotor frontRightWheel;
    public DcMotor backRightWheel;

    public DcMotor viperSlideMotor;

    public DcMotor intakeExtender;
    public DcMotor sweeper;

    // 4 Servos
    public Servo sweeperRotator;

    public Servo dumperServo;

    public Servo clawLeft;
    public Servo clawRight;

    public Servo rightLift;
    public Servo leftLift;

    // 2 Touch sensors
    public TouchSensor touchLeft;
    public TouchSensor touchRight;

    public ViperSlide viperSlide;
    public Intake intake;
    public Odometry odometry;
    public Drive drive;
    //public Movement movement;
    public MovementBuilder movement;

    public ElapsedTime timer;

    private void autoBrake(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void unbrakeMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

//    public void floatAllWheels() {
//        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//    }


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

    public void initWheels() {
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeft");
        autoBrake(frontLeftWheel);
        reverse(frontLeftWheel);

        backLeftWheel = hardwareMap.get(DcMotor.class, "backLeft");
        autoBrake(backLeftWheel);

        frontRightWheel = hardwareMap.get(DcMotor.class, "frontRight");
        autoBrake(frontRightWheel);

        backRightWheel = hardwareMap.get(DcMotor.class, "backRight");
        autoBrake(backRightWheel);
    }

    public void initHardware() {
        initWheels();

        intakeExtender = hardwareMap.get(DcMotor.class, "intakeMotor");
        runToPositionMode(intakeExtender);
        autoBrake(intakeExtender);
        reverse(intakeExtender);

        viperSlideMotor = hardwareMap.get(DcMotor.class, "linearSlide");
        reverse(viperSlideMotor);
        runToPositionMode(viperSlideMotor);

        sweeper = hardwareMap.get(DcMotor.class, "sweeper");

        sweeperRotator = hardwareMap.get(Servo.class, "sweeperRotator");

        dumperServo = hardwareMap.get(Servo.class, "dumperServo");

        touchLeft = hardwareMap.get(TouchSensor.class, "touchLeft");
        touchRight = hardwareMap.get(TouchSensor.class, "touchRight");

        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        leftLift = hardwareMap.get(Servo.class, "leftLift");
        rightLift = hardwareMap.get(Servo.class, "rightLift");

        initOdometry();
    }

    public void initOdometry() {
        odometry = new Odometry(this);
    }

    public void initRobot() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        initHardware();

        viperSlide = new ViperSlide(this);
        intake = new Intake(this);
        drive = new Drive(this);
        //movement = new Movement(this);
        movement = new MovementBuilder(this);

        // disable servos
        //clawLeft.getController().pwmDisable();

        initServos();

        // make intake arm always up unless doing basket/bucket
    }

    public void initAllButServos() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        initHardware();

        viperSlide = new ViperSlide(this);
        intake = new Intake(this);
        drive = new Drive(this);
        //movement = new Movement(this);
        movement = new MovementBuilder(this);
    }

    public void initServos() {
        intake.armIn();
        viperSlide.bucketDown();
        viperSlide.clawOut();
        viperSlide.liftersIn();
    }

    public void loopUpdate() {
        viperSlide.loopUpdate();
        intake.loopUpdate();
        movement.target.updatePosition();
    }

    public boolean gamepadHasInterrupted() {
        return gamepad1.dpad_down;
    }

    /**
     * Makes sure opMode is running and that the controller has not canceled the movement.
     */
    public boolean isNotInterrupted() {
        return !gamepadHasInterrupted() && opModeIsActive();
    }
}
