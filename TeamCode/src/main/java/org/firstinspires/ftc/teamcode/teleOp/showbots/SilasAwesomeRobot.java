package org.firstinspires.ftc.teamcode.teleOp.showbots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SilasAwesomeRobot extends LinearOpMode {
    double ARM_DOWN = -0.5;
    double ARM_UP = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
       // When you you click init on driver station but haven't clicked play
        DcMotor leftMotor= hardwareMap.get(DcMotor.class,"lefttread");
        DcMotor rightMotor= hardwareMap.get(DcMotor.class,"righttread");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo claw=hardwareMap.get(Servo.class,"claw");

        // TODO: declare another servo for the claw elbow
        Servo clawPincher =hardwareMap.get(Servo.class,"clawPincher");
        waitForStart();
       // After you click the play button

        while(opModeIsActive()) {
            // loops hundreds of times per second
            leftMotor.setPower(-gamepad1.right_stick_y * .4);
            rightMotor.setPower(-gamepad1.left_stick_y * .4);

            // TODO: what control will be used to set the claw elbow position
            if (gamepad1.triangle) {
                claw.setPosition(0);
            }
            if (gamepad1.x) {
                claw.setPosition(.25);
            }
            // TODO: what control will be used to set the claw position
            if (gamepad1.right_trigger>0.5) {
                clawPincher.setPosition(0);
            }
            if (gamepad1.left_trigger>0.5) {
                clawPincher.setPosition(.5);
            }

            // 1
            //
        }
    }
}