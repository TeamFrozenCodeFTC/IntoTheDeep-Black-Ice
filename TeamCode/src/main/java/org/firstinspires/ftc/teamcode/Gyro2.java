package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro2 {

    private double angle = 0;
    private double storedAngle = 0;
    private double last = 0;
    BNO055IMU gyro;
    LinearOpMode opMode;

    public Gyro2(BNO055IMU gyro, LinearOpMode op) {
        this.gyro = gyro;
        this.opMode = op;
    }

    public void reset() {
        angle = 0;
        last = read();
    }

    public void resetRotation(int degrees) {
        angle = degrees;
        last = read();
    }

    public void store() {
        storedAngle = angle;
    }

    public void recall() {
        angle = storedAngle;
    }

    public double read() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // are we going to need accumulative degrees?
    public double getAngle() {
        double current = read();

        if(last >= 0 && current >= 0) {
            angle += current - last;
            storedAngle += current - last;
        } else if (last < 0 && current < 0) {
            angle += current - last;
            storedAngle += current - last;
        } else if (last >= 90 && current <= -90) {
            angle += current + 180 + 180 - last;
            storedAngle += current + 180 + 180 - last;
        } else if (last >= 0 && last < 90 && current > -90 && current < 0) {
            angle -= last - current;
            storedAngle -= last - current;
        } else if (current >= 90 && last <= -90) {
            angle -= last + 180 + 180 - current;
            storedAngle -= last + 180 + 180 - current;
        } else if (current >= 0 && current < 90 && last > -90 && last < 0) {
            angle += current - last;
            storedAngle += current - last;
        } else {
        }
        last = current;

        return angle;
    }

    public void startGyro(){
        // The imu complains if you try to initialize it twice so check if it is already initialized
        //if(!gyro.isGyroCalibrated()) {

            // Set the parameters to initialize the imu
            // This was referenced from http://stemrobotics.cs.pdx.edu/node/7265
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // send a message to the phone that the gyro is being initialized
            opMode.telemetry.addData("Gyro Mode:", "calibrating...");
            opMode.telemetry.update();

            // initialize the imu with the parameters
            gyro.initialize(parameters);


            // wait for rhe calibration to complete
            while (!gyro.isGyroCalibrated()) {
                // do noting... just wait
                opMode.idle();
            }

            opMode.telemetry.addData("Gyro Mode:", "ready");
            opMode.telemetry.update();
        //}

        // send a message to the phone about the calibration status
        //opMode.telemetry.addData("Gyro Status", gyro.getCalibrationStatus().toString());
        //opMode.telemetry.update();
    }
}