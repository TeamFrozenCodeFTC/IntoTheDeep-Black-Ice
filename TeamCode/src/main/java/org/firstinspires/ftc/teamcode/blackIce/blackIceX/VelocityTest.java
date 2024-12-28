package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class VelocityTest extends RobotMovement {

    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(0,0,0);
        targetHeading = 0;
        targetX = 0;//TILE * 2 + EDGE_OF_TILE;
        targetY = 0;

        //goStraightForSeconds(0.4, 1);

        targetY = 0;//TILE + EDGE_OF_TILE;

        updatePosition();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive()) {
            double stoppingDistance = odometry.yBrakingDistance + odometry.xBrakingDistance;

//            if (distanceToTarget <= stoppingDistance && odometry.velocity > CONTROLLABLE_VELOCITY) {
//                // try braking using negative powers
//                updatePosition();
//                brake();
//                continue;
//            }
//
//            double rotation = odometry.heading - 90;
//
//            double currentXVel = Math.signum(odometry.xVelocity) *(
//                    0.00130445 * Math.pow(odometry.xVelocity, 2)
//                            + 0.0644448 * Math.abs(odometry.xVelocity) + 0.0179835);
//            double currentYVel = Math.signum(odometry.yVelocity) * (
//                    0.00130445 * Math.pow(odometry.yVelocity, 2)
//                            + 0.0644448 * Math.abs(odometry.yVelocity) + 0.0179835);
//
//            double x1 = -(targetX - odometry.x - currentXVel);
//            double y1 = (targetY - odometry.y) - currentYVel;
//
//            double cos = Math.cos(Math.toRadians(rotation));
//            double sin = Math.sin(Math.toRadians(rotation));
//            double x = x1 * cos - y1 * sin;
//            double y = x1 * sin + y1 * cos;
//
//// * getSign(odometry.xVelocity)
//            // negative?
//
//            telemetry.addData("currentXVel", currentXVel);
//            telemetry.addData("currentYVel", currentYVel);
//            telemetry.addData("target x", (targetX - odometry.x));
//            telemetry.addData("target y", (targetY - odometry.y));
//
//            telemetry.addData("slide", y);
//            telemetry.addData("forward", x);
//
//            double[] powers = normalize(new double[] {(y-x)/10, (x+y)/10, (x+y)/10, (y-x)/10});
//
//            telemetry.addData("frontLeft", powers[0]);
//            telemetry.addData("backLeft", powers[1]);
//            telemetry.addData("frontRight", powers[2]);
//            telemetry.addData("backRight", powers[3]);
//
//            powerWheels(powers);


            double rotation = -odometry.heading;

            double currentXVel = Math.signum(odometry.xVelocity) *(
                    0.00130445 * Math.pow(odometry.xVelocity, 2)
                            + 0.0644448 * Math.abs(odometry.xVelocity) + 0.0179835);
            double currentYVel = Math.signum(odometry.yVelocity) * (
                    0.00130445 * Math.pow(odometry.yVelocity, 2)
                            + 0.0644448 * Math.abs(odometry.yVelocity) + 0.0179835);

            double x1 = targetX - odometry.x - currentXVel;
            double y1 = (targetY - odometry.y) - currentYVel;

            double cos = Math.cos(Math.toRadians(rotation));
            double sin = Math.sin(Math.toRadians(rotation));
            double x = x1 * cos - y1 * sin;
            double y = x1 * sin + y1 * cos;

// * getSign(odometry.xVelocity)
            // negative?

            telemetry.addData("currentXVel", currentXVel);
            telemetry.addData("currentYVel", currentYVel);
            telemetry.addData("target x", (targetX - odometry.x));
            telemetry.addData("target y", (targetY - odometry.y));

            telemetry.addData("slide", y);
            telemetry.addData("forward", x);

            double[] powers = normalize(new double[] {(x-y)/10, (x+y)/10, (x+y)/10, (x-y)/10});

            telemetry.addData("frontLeft", powers[0]);
            telemetry.addData("backLeft", powers[1]);
            telemetry.addData("frontRight", powers[2]);
            telemetry.addData("backRight", powers[3]);

            powerWheels(powers);


//            telemetry.addData("x1", x1);
//            telemetry.addData("y1", y1);

//            telemetry.addData("XVel", odometry.xVelocity);
//            telemetry.addData("YVel", odometry.yVelocity);
//            telemetry.addData("YVel", odometry.velocity);
//            telemetry.addData("heading", odometry.heading);
//            telemetry.addData("total", odometry.xVelocity + odometry.yVelocity);
//
//            telemetry.addData("x", x);
//            telemetry.addData("y", y);

            //forceTowardTarget2();
            //goTowardTarget();

            telemetry.update();
            updatePosition();
        }
        //holdFor(3);
    }
}