package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class MeasurePosition extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        sweeperRotator.getController().pwmDisable();

        drive.zeroPowerFloat();

        while (opModeIsActive()) {
            odometry.update();
            telemetry.addData("x -> ", odometry.x);
            telemetry.addData("y  ^ ", odometry.y);
            telemetry.addData("heading", odometry.heading);

            telemetry.update();
        }
        
    }
}