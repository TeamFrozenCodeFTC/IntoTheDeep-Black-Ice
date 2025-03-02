package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.Constants;
import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.Point;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

/**
 * Takes a path of points and calculates the heading at each point.
 */
public class Path extends MovementBuilder<Path> {
    final double[][] points;
    double[] headings;

    public double constantHeading = 0;
    double headingOffset = 0;
    boolean isConstantHeading = false;

    double endingHeading;
    double[] endPoint;

    private final double headingBetweenEndPoints;

    /**
     * Create a path of points the robot follows.
     */
    public Path(double[][] points) {
        this.points = points;
        this.endPoint = points[points.length - 1];

        this.headings = calculateHeadings();

        this.headingBetweenEndPoints = Math.toDegrees(
            Math.atan2(endPoint[1] - points[points.length - 2][1],
                endPoint[0] - points[points.length - 2][0])
        );
    }

    // TODO try derivatives of the path to get the heading
    private double[] calculateHeadings() {
        double[] headings = new double[this.points.length];

        for (int i = 0; i < this.points.length; i++) {
            double[] point = this.points[i];

            if (this.isConstantHeading) {
                headings[i] = this.constantHeading;
                continue;
            }

            double[] pointHeading;
            if (i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING >= this.points.length) {
                pointHeading = this.endPoint;
            } else {
                pointHeading = this.points[i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING];
            }

            headings[i] = Math.toDegrees(
                Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
            ) + this.headingOffset;
        }
        return headings;
    }

    /**
     * Make the robot have a constant heading along the path.
     */
    public Path setConstantHeading(double newConstantHeading) {
        isConstantHeading = true;
        constantHeading = newConstantHeading;
        endingHeading = constantHeading;
        return this;
    }

    /**
     * Make the robot have a offset heading along the path.
     */
    public Path setHeadingOffset(double newHeadingOffset) {
        isConstantHeading = false;
        headingOffset = newHeadingOffset;
        endingHeading = headingBetweenEndPoints + headingOffset;
        return this;
    }

    public Path setLinearHeadingInterpolation(double endingHeading) {

        return this; // finish percent by
    }

    private int i = 1;
    private boolean holding = false;
    private boolean finished = false;

    private void update() {
        if (holding) {
            stopAtEndPosition.update();
            if (stopAtEndPosition.isFinished()) {
                finished = true;
            }
            return;
        }

        Target.updatePosition();

        double[] point = this.points[i];

        Follower.telemetry.addData("i", i);
        Follower.telemetry.update();

        double inchesLeftOnPath = (this.points.length - i) * Constants.Curve.INCHES_PER_POINT;
        if (inchesLeftOnPath < Util.getVectorMagnitude(
            Odometry.xBrakingDistance, Odometry.yBrakingDistance) + 2) {
            stopAndHoldEndPoint();
            return;
        }

        new Point(point[0], point[1], this.headings[i])
            .copyProperties(this)
            .moveThrough()
            .build()
            .waitForMovement();

        this.i += 1;
        if (i >= this.points.length) {
            stopAndHoldEndPoint();
        }
    }

    @Override
    protected Path getThis() {
        return this;
    }

    private Movement stopAtEndPosition;

    private void stopAndHoldEndPoint() {
        stopAtEndPosition = new Point(this.endPoint[0], this.endPoint[1], this.endingHeading)
            .copyProperties(this)
            .stopAtPosition()
            .build();
        stopAtEndPosition.start();

        holding = true;
    }

    public Movement build() {
        this.headings = calculateHeadings();
        return new Movement(() -> finished, this::start, this::finish, this::update);
    }

    private void start() {
        i = 1;
        holding = false;
        finished = false;

        Target.setTarget(Target.previousHeading, Path.this.points[0][0], Path.this.points[0][1]); //?
    }

    private void finish() {
        Drive.zeroPowerBrakeMode();
        Drive.zeroPower();
    }
}