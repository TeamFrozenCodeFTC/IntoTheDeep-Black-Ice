package org.firstinspires.ftc.teamcode.autonomous.custom;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// A parent class to all operation modes. Contains the Robot's Hardware but also LinearOpMode.
public abstract class Robot extends LinearOpMode {
    public DcMotor frontLeftWheel;
    public DcMotor backLeftWheel;
    public DcMotor frontRightWheel;
    public DcMotor backRightWheel;
    public BNO055IMU imu;

    void autoBrake(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void unbrakeMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void reverse(DcMotor motor) {
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initWheels() {
        frontLeftWheel = hardwareMap.get(DcMotor.class, "frontLeft");
        //autoBrake(frontLeftWheel);
        reverse(frontLeftWheel);

        backLeftWheel = hardwareMap.get(DcMotor.class, "backLeft");
        //autoBrake(backLeftWheel);

        frontRightWheel = hardwareMap.get(DcMotor.class, "frontRight");
        //autoBrake(frontRightWheel);
        reverse(frontRightWheel);

        backRightWheel = hardwareMap.get(DcMotor.class, "backRight");
        //autoBrake(backRightWheel);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // send a message to the phone that the gyro is being initialized
        telemetry.addData("Gyro Mode:", "calibrating...");
        telemetry.update();

        // initialize the imu with the parameters
        imu.initialize(parameters);

        // wait for rhe calibration to complete
        while (!imu.isGyroCalibrated()) {
            // do noting... just wait
            idle();
        }

        telemetry.addData("Gyro Mode:", "ready");
        telemetry.update();
    }
}
