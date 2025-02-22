package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.Robot;

public class Curve {
    public static int LOOK_AHEAD_HEADING = 1;
    double[][] points;

    public double constantHeading = 0;
    public double headingOffset = 0;
    public boolean isConstantHeading = false;

    public Curve(double[][] points) {
        this.points = points;
    }

    public Curve setConstantHeading(double newConstantHeading) {
        isConstantHeading = true;
        constantHeading = newConstantHeading;
        return this;
    }

    public Curve setHeadingOffset(double newHeadingOffset) {
        isConstantHeading = false;
        headingOffset = newHeadingOffset;
        return this;
    }

    public void run() {
        Robot robot = Robot.instance;

        // Set previous Point
        robot.movement.target.setTarget(robot.odometry.heading, points[0][0], points[0][1]);

        double[] endPoint = points[points.length - 1];

        for (int i = 1; i <= points.length; i++) {
            robot.loopUpdate();

            double[] point = points[i];

            if (i == points.length - 6) { // subtract braking distance
                break;
            }

            double[] pointHeading;
            if (i + LOOK_AHEAD_HEADING >= points.length) {
                pointHeading = endPoint;
            }
            else {
                pointHeading = points[i + LOOK_AHEAD_HEADING];
            }

            double targetHeading;
            if (isConstantHeading) {
                targetHeading = constantHeading;
            } else {
                targetHeading = Math.toDegrees(
                    Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
                ) + headingOffset;
            }

            robot.movement.buildMovement(targetHeading, point[0], point[1])
                .moveThrough2()
                .run();

            //robot.movement.moveThrough(targetHeading, point[0], point[1]); // use different func
        }

        robot.telemetry.addData("driving to endpoint", 0);
        robot.telemetry.update();

        robot.loopUpdate();

        double targetHeading;
        if (isConstantHeading) {
            targetHeading = constantHeading;
        } else {
            targetHeading = Math.toDegrees(
                Math.atan2(endPoint[1] - points[points.length - 2][1], endPoint[0] - points[points.length - 2][0])
            ) + headingOffset;
        }
        robot.movement.stopAtPosition(targetHeading, endPoint[0], endPoint[1]);
        robot.drive.zeroPower();
    }
}// TODO
// TODO make tests for curves, lines, and max velocities, etc
