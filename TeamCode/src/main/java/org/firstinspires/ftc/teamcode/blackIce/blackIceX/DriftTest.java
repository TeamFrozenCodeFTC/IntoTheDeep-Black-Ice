package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class DriftTest extends RobotMovement {
    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(0,0,0);
        targetHeading = 0;
        targetX = TILE * 2 + EDGE_OF_TILE;
        targetY = 0;

        goStraightForSeconds(0.4, 1);

        targetY = TILE + EDGE_OF_TILE;

        updatePosition();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
            double stoppingDistance = 0;

            if (distanceToTarget <= stoppingDistance && odometry.velocity > CONTROLLABLE_VELOCITY) {
                // try braking using negative powers
                updatePosition();
                brake();
                continue;
            }

            double rotation = odometry.heading - 90;
            double x1 = -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN;
            double y1 = (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN;

            double cos = Math.cos(Math.toRadians(rotation));
            double sin = Math.sin(Math.toRadians(rotation));
            double x = x1 * cos - y1 * sin;
            double y = x1 * sin + y1 * cos;

            powerWheels(applyCorrection(normalize(new double[] {y-x, y+x, y+x, y-x})));

            telemetry.addData("velocity", odometry.xVelocity);
            telemetry.addData("x", odometry.x);
            telemetry.addData("y", odometry.y);
            telemetry.addData("delta", timer.milliseconds());

            telemetry.update();
            updatePosition();
            timer.reset();
        }
        holdFor(3);
    }
}