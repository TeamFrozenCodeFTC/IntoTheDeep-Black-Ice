package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class DirectionalSpeedTest extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(0,0,0);

        updatePosition();

        goStraightForSeconds(0.7, 1);
        updatePosition();
        double maxVelocityForwards = odometry.xVelocity;
        brakeForSeconds(2);

        goStraightForSeconds(0.7, -1);
        updatePosition();
        double maxVelocityBackwards = odometry.xVelocity;
        brakeForSeconds(2);

        slideForSeconds(0.7, 1);
        updatePosition();
        double maxVelocitySlidePos = odometry.yVelocity;
        brakeForSeconds(2);

        slideForSeconds(0.7, -1);
        updatePosition();
        // 65
        // 65
        // 55
        // 55
        // 58

        double maxVelocitySlideNeg = odometry.yVelocity;
        brakeForSeconds(2);

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 0.7) {
            updatePosition();

            powerWheels(new double[] {1, 0, 0, 1});
        }
        updatePosition();
        double maxDiagonalVel = odometry.velocity;
        brakeForSeconds(2);

        telemetry.addData("velocity forwards", maxVelocityForwards);
        telemetry.addData("velocity backwards", maxVelocityBackwards);
        telemetry.addData("velocity slide pos", maxVelocitySlidePos);
        telemetry.addData("velocity slide neg", maxVelocitySlideNeg);
        telemetry.addData("velocity diagonal vel", maxDiagonalVel);
        telemetry.update();
        sleep(30000);
    }
}