package org.firstinspires.ftc.teamcode.teleOp.showbots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class TwoWheelRobot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // When you you click init on driver station but haven't clicked play
        DcMotor forwardWheel = hardwareMap.get(DcMotor.class,"forwardWheel");
        DcMotor turningWheel = hardwareMap.get(DcMotor.class,"turningWheel");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class,"imu");

        imu.initialize(new BNO055IMU.Parameters());

        waitForStart();
        // After you click the play button

        while(opModeIsActive()) {
            // loops hundreds of times per second

            double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            telemetry.addData("angle", angle);
            telemetry.update();

            forwardWheel.setPower(gamepad1.left_stick_y);
            turningWheel.setPower(gamepad1.right_stick_x + (0 - angle) * 0.05);


        }
    }
}
