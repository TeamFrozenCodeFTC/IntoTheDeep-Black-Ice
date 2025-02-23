package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.Drive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.blackIce.Constants;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

public class Path extends Movement {
    double[][] points;

    public double constantHeading = 0;
    public double headingOffset = 0;
    public boolean isConstantHeading = false;

    public Path(double[][] points) {
        this.points = points;
    }

    public Path setConstantHeading(double newConstantHeading) {
        isConstantHeading = true;
        constantHeading = newConstantHeading;
        return this;
    }

    public Path setHeadingOffset(double newHeadingOffset) {
        isConstantHeading = false;
        headingOffset = newHeadingOffset;
        return this;
    }

    public void runCurve(Movement movementBuild) {
        Robot robot = Robot.robot;

        // Set previous Point
        Target.setTarget(Odometry.heading, points[0][0], points[0][1]);

        double[] endPoint = points[points.length - 1];

        for (int i = 1; i <= points.length; i++) {
            robot.loopUpdate();

            robot.telemetry.addData("i", i);
            robot.telemetry.addData("points", points.length);
            robot.telemetry.update();

            double[] point = points[i];

            double inchesLeftOnPath = (points.length - i) * Constants.Curve.INCHES_PER_POINT;
            if (inchesLeftOnPath < Odometry.brakingDistance + 2) { // multiply by POINTS_PER_INCH
                break;
            }

            double[] pointHeading;
            if (i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING >= points.length) {
                pointHeading = endPoint;
            }
            else {
                pointHeading = points[i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING];
            }

            double targetHeading;
            if (isConstantHeading) {
                targetHeading = constantHeading;
            } else {
                targetHeading = Math.toDegrees(
                    Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
                ) + headingOffset;
            }

            robot.telemetry.addData("going", 1);
            robot.telemetry.update(); // TODO run and test telemetry

            // Hacky Solution for allowing things to be build for curve
            Target.setTarget(targetHeading, point[0], point[1]);
            robot.loopUpdate();
            movementBuild.curve = null; // otherwise gets stuck in loop
            movementBuild.moveThrough().noBrakeAfter().run();

//            new Movement(point[0], point[1], targetHeading)
//                .moveThrough2()
//                .noBrakeAfter()
//                .run();
        }

        robot.telemetry.addData("to target pos", 1);
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

        Target.setTarget(targetHeading, endPoint[0], endPoint[1]);
        robot.loopUpdate();
        movementBuild.curve = null; // otherwise gets stuck in loop
        movementBuild.stopAtPosition().run();

        // Movement.stopAtPosition(targetHeading, endPoint[0], endPoint[1]);

        Drive.zeroPower();
    }
}
// TODO optimize and make subfolders