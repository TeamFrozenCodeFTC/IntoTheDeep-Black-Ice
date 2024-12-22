package org.firstinspires.ftc.teamcode.blackIce.blackIceX;


import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class VelocityTest extends RobotMovement {
    public static double[] adjustTargetDirection(double[] targetVector, double[] velocityVector) {
        // Combine the target vector with the velocity vector
        double adjustedX = (targetVector[0] + velocityVector[0] * 0.03);
        double adjustedY = (targetVector[1] + velocityVector[1] * 0.03);

        // TODO switch x and y, multiply by time

//        double adjustedX = (targetVector[0]);
//        double adjustedY = (targetVector[1] );

        // Return the adjusted vector as-is without normalizing
        return new double[] { adjustedX, adjustedY };
    }

    @Override
    public void runOpMode() {
        initOdometry();
        initWheels();
        waitForStart();

        odometry.setPosition(0,0,0);
        targetHeading = 0;
        targetX = 30;
        targetY = 0;

        //goStraightForSeconds(0.5, 1);

        goStraightForSeconds(0.4, 1);

        targetY = 30;

        updatePosition();

        while (opModeIsActive() && isNotWithinErrorMargin(defaultErrorMargin)) {
            double stoppingDistance = estimateStoppingDistance();

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
            double xx = x1 * cos - y1 * sin;
            double yy = x1 * sin + y1 * cos;

            double maxDirectionalVelocity = Math.min(odometry.xVelocity, odometry.yVelocity);

            double[] target = adjustTargetDirection(
                    new double[] {xx,yy},
                    new double[] {odometry.xVelocity-maxDirectionalVelocity, odometry.yVelocity-maxDirectionalVelocity}
            );
            double x = target[0];
            double y = target[1];

            powerWheels(applyCorrection(normalize(new double[] {y-x, y+x, y+x, y-x})));

            telemetry.addData("velocity", odometry.xVelocity);
            telemetry.addData("xV", odometry.xVelocity-maxDirectionalVelocity);
            telemetry.addData("yV", odometry.yVelocity-maxDirectionalVelocity);
            telemetry.addData("x", odometry.x);
            telemetry.addData("y", odometry.y);

            telemetry.update();
            updatePosition();
        }
        holdFor(3);
    }
}