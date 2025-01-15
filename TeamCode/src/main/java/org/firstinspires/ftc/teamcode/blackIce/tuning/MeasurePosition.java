package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Tuning")
public class MeasurePosition extends Robot {
    public static int roundToNearest15(int num) {
        return Math.round(num / 15.0f) * 15;
    }

    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        while (opModeIsActive()) {
            odometry.update();
            telemetry.addData("x -> ", odometry.x);
            telemetry.addData("y  ^ ", odometry.y);
            telemetry.addData("heading", roundToNearest15((int) odometry.heading));

            telemetry.update();
        }
    }
}