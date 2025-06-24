package org.firstinspires.ftc.teamcode.teleOp.showbots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class SilasServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo hammer=hardwareMap.get(Servo.class,"Hammer");
        DcMotor leftMotor= hardwareMap.get(DcMotor.class,"lefttread");
        DcMotor rightMotor= hardwareMap.get(DcMotor.class,"righttread");
       waitForStart();
       hammer.setPosition(0.5);
       sleep(1000);
       hammer.setPosition(1);
        sleep(1000);
        hammer.setPosition(0.5);
        sleep(1000);
        hammer.setPosition(1);
        leftMotor.setPower(0.5);
        //rightMotor.setPower(0.5);
        while (opModeIsActive()) {

       }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
