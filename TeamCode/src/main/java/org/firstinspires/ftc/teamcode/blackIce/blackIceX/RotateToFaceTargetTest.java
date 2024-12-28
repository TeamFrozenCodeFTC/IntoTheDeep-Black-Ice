package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class RotateToFaceTargetTest extends RobotMovement {

    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(90,TILE * 2 - 5, 3);

        targetHeading = -90;
        targetX = 0;
        targetY = 30;

        updatePosition();

        while (opModeIsActive() && isNotWithinErrorMargin(wideErrorMargin)) {
            double rotation = -odometry.heading;

            double x1 = (targetX - odometry.x);
            double y1 = (targetY - odometry.y);

            double cos = Math.cos(Math.toRadians(rotation));
            double sin = Math.sin(Math.toRadians(rotation));
            double x = (x1 * cos - y1 * sin) / 5;
            double y = (x1 * sin + y1 * cos) / 5;

            double[] powers = normalize(new double[] {x-y, x+y, x+y, x-y});

            double targetAngleCorrection;

            if (distanceToTarget < 25) {
                targetAngleCorrection = getTurnCorrection();
            }
            else {
                double degrees = Math.toDegrees(Math.atan2(y1, x1)) - targetHeading + 90;
                targetAngleCorrection = simplifyAngle(degrees - odometry.heading) * 0.015;
                telemetry.addData("targetAngleCorrection", targetAngleCorrection);
                telemetry.addData("targetAngle", degrees);
                telemetry.addData("heading", odometry.heading);
                telemetry.update();
            }

            for (int i = 0; i < powers.length; i++) {
                double total = powers[i];
                if (i < 2) {
                    total -= targetAngleCorrection;
                }
                else {
                    total += targetAngleCorrection;
                }
                powers[i] = clampPower(total) * MAX_POWER * powerMult;
            }

            powerWheels(normalize(powers));

            updatePosition();
        }

        brakeForSeconds(3);

        updatePosition();


//        // Move away from submersible
//        accelerate(-90, 4, TILE + EDGE_OF_TILE - 1.5);
//        // Turn and Go towards sample
////        LINEAR_INCH_SLOW_DOWN = 10;
//        accelerate(-180, TILE, TILE + EDGE_OF_TILE - 3);
//
//        // 1st sample
//        directionChange(-180, TILE * 2 - 8, TILE + EDGE_OF_TILE + HALF_OF_ROBOT - 2);
//        quickBrakeTo( 90, TILE * 2 - 5, 3);

//
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
//            double stoppingDistance = 0;
//
//            if (distanceToTarget <= stoppingDistance && odometry.velocity > CONTROLLABLE_VELOCITY) {
//                // try braking using negative powers
//                updatePosition();
//                brake();
//                continue;
//            }
//
//            double rotation = odometry.heading - 90;
//            double x1 = -(targetX - odometry.x) / LINEAR_INCH_SLOW_DOWN;
//            double y1 = (targetY - odometry.y) / LINEAR_INCH_SLOW_DOWN;
//
//            double cos = Math.cos(Math.toRadians(rotation));
//            double sin = Math.sin(Math.toRadians(rotation));
//            double x = x1 * cos - y1 * sin;
//            double y = x1 * sin + y1 * cos;
//
//            powerWheels(applyCorrection(normalize(new double[] {y-x, y+x, y+x, y-x})));
//
//            telemetry.addData("velocity", odometry.xVelocity);
//            telemetry.addData("x", odometry.x);
//            telemetry.addData("y", odometry.y);
//            telemetry.addData("delta", timer.milliseconds());
//
//            telemetry.update();
//            updatePosition();
//            timer.reset();
//        }
//        holdFor(3);
    }
}