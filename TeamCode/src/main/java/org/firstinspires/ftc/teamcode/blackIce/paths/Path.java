package org.firstinspires.ftc.teamcode.blackIce.paths;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.Constants;
import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.BaseMovementBuild;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuild;
import org.firstinspires.ftc.teamcode.blackIce.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

/**
 * Takes a path of points and calculates the desired heading at each point.
 */
public class Path extends BaseMovementBuild<Path> {
    final double[][] points;
    double[] headings;

    double[] endPoint;
    double endingHeading;

    boolean stopAtEndOfPath = true;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Create a path of points the robot follows.
     */
    public Path(double[][] points) {
        this.points = points;
        this.endPoint = new double[] {
            this.points[points.length - 1][0],
            this.points[points.length - 1][1]
        };
//        this.headings = setConstantHeading(0);
        this.headings = calculateHeadings(0);
        this.endingHeading = this.headings[headings.length - 1];
    }

    // TODO try derivatives of the path to get the heading (make sure to not force path continuity)
    private double[] calculateHeadings(double headingOffset) {
        double[] headings = new double[this.points.length];

        for (int i = 0; i < this.points.length; i++) {
            double[] point = this.points[i];

            double[] pointHeading;
            if (i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING >= this.points.length) {
                pointHeading = this.endPoint;
            } else {
                pointHeading = this.points[i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING];
            }

            headings[i] = Math.toDegrees(
                Math.atan2(pointHeading[1] - point[1], pointHeading[0] - point[0])
            ) + headingOffset;
        }
        return headings;
    }

    /**
     * Make the robot have a constant heading along the path.
     */
    public Path setConstantHeading(double newConstantHeading) {
        headings = new double[this.points.length];
        for (int i = 0; i < this.points.length; i++) {
            headings[i] = newConstantHeading;
        }
        return this;
    }

    /**
     * Make the robot continue its momentum at the end of the path.
     * <p>
     * Note: if continued to update after the path has finished,
     * the robot will overshoot the end position.
     */
    public Path moveThroughPath() {
        stopAtEndOfPath = false;
        return this;
    }

    /**
     * Make the robot stop at the end of the path.
     */
    public Path stopAtEndOfPath() {
        stopAtEndOfPath = true;
        return this;
    }

    /**
     * Make the robot have a offset heading along the path.
     */
    public Path setHeadingOffset(double newHeadingOffset) {
        headings = calculateHeadings(newHeadingOffset);
        return this;
    }

    /**
     * Make the robot smoothly transition from one heading to another.
     */
    public Path setLinearHeadingInterpolation(double startingHeading, double endingHeading) {
        headings = new double[this.points.length];
        for (int i = 0; i < this.points.length; i++) {
            double t = (double) i / (this.points.length - 1);
            headings[i] = startingHeading + t * (endingHeading - startingHeading);
        }
        return this;
    }

    private int i = 1;
    private boolean holdingEndPosition = false;
    private boolean finished = false;

    private void update() {
        if (holdingEndPosition) {
            if (holdEndPosition.isFinished()) {
                finished = true;
            }
            holdEndPosition.update();
            return;
        }

        Target.updatePosition();

        double[] point = this.points[i];

        double inchesLeftOnPath = (this.points.length - i) * Constants.Curve.INCHES_PER_POINT;
        if (inchesLeftOnPath < Util.getVectorMagnitude(
            Odometry.xBrakingDistance, Odometry.yBrakingDistance) + 2) {
            startHoldingEndPosition();
            return;
        }

        MovementBuilder.moveThrough(point[0], point[1], this.headings[i])
            .copyProperties(this)
            .build()
            .waitForMovement();

        this.i += 1;
        if (i >= this.points.length) {
            startHoldingEndPosition();
        }
    }

    @Override
    protected Path getThis() {
        return this;
    }

    private MovementBuild endPosition;
    private Movement holdEndPosition;

    private void startHoldingEndPosition() {
        holdEndPosition = endPosition.copyProperties(this).build();
        holdEndPosition.start();
        // TODO cannot copy all properties? make .copyProperties copy all properties depending on
        // both types

        holdingEndPosition = true;
    }

    /**
     * Finish building the path. This should be the last method called.
     * If you want to inherit methods
     */
    public Movement build() {
        this.endingHeading = this.headings[headings.length - 1];

        if (stopAtEndOfPath) {
            endPosition = MovementBuilder.stopAtPosition(
                endPoint[0],
                endPoint[1],
                endingHeading
            );
        }
        else {
            endPosition = MovementBuilder.moveThrough(
                endPoint[0],
                endPoint[1],
                endingHeading
            );
        }
        return new Movement(() -> finished, this::start, this::finish, this::update);
    }

    private void start() {
        i = 1;
        holdingEndPosition = false;
        finished = false;
        Target.setTarget(Target.previousHeading, Path.this.points[0][0], Path.this.points[0][1]); //?
        timer.reset();
    }

    private void finish() {
        Drive.zeroPowerBrakeMode();
        Drive.zeroPower();
    }

//    @Override
//    public Path copyProperties(MovementBuild<?> build) {
//        return super.copyProperties(build)
//            .setConstantHeading()
//    }
}