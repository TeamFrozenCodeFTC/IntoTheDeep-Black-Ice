package org.firstinspires.ftc.teamcode.teleOp.showbots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.Util;

@TeleOp
public class SilasAwesomeRobot extends LinearOpMode {
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

       // When you you click init on driver station but haven't clicked play
        DcMotor leftMotor= hardwareMap.get(DcMotor.class,"lefttread");
        DcMotor rightMotor= hardwareMap.get(DcMotor.class,"righttread");

        Servo claw= hardwareMap.get(Servo.class,"claw");

        Servo clawPincher= hardwareMap.get(Servo.class,"clawPincher");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
       // After you click the play button

        claw.setPosition(0);

        while(opModeIsActive()) {
            telemetry.addData("angle", Util.simplifyAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
                .firstAngle));
            telemetry.update();
            if (Util.simplifyAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
                    .firstAngle) > 0) {
                leftMotor.setPower(-gamepad1.left_stick_y);
                rightMotor.setPower(-gamepad1.right_stick_y);
            }
            else {
                leftMotor.setPower(gamepad1.left_stick_y);
                rightMotor.setPower(gamepad1.right_stick_y);
            }

            if (gamepad1.right_bumper) {
                claw.setPosition(.5);
            }
            if (gamepad1.left_bumper) {
                claw.setPosition(0);
            }

            if (gamepad1.x) {
                clawPincher.setPosition(0);
            }
            else {
                clawPincher.setPosition(.5);
            }
       }
        // after you click the stop button
    }
}
