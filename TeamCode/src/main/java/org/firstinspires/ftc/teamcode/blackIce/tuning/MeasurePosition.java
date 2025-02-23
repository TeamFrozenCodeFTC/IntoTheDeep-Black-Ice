package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class MeasurePosition extends Robot {
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        sweeperRotator.getController().pwmDisable();

        Drive.zeroPowerFloat();

        while (opModeIsActive()) {
            Odometry.update();
            telemetry.addData("x -> ", Odometry.x);
            telemetry.addData("y  ^ ", Odometry.y);
            telemetry.addData("heading", Odometry.heading);

            telemetry.update();
        }
        
    }
}