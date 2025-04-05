package org.firstinspires.ftc.teamcode.blackIce.paths;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blackIce.Constants;
import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.DriveCorrection;
import org.firstinspires.ftc.teamcode.blackIce.Follower;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuild;
import org.firstinspires.ftc.teamcode.blackIce.movement.MovementBuilder;
import org.firstinspires.ftc.teamcode.blackIce.movement.Movement;
import org.firstinspires.ftc.teamcode.blackIce.Target;
import org.firstinspires.ftc.teamcode.blackIce.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

/**
 * Takes a path of points and calculates the desired heading at each point.
 */
public class Path implements Cloneable {
    public final double[][] points;
    double[] headings;

    double[] endPoint;
    double endingHeading;

    boolean stopAtEndOfPath = true;
    MovementProperties moveThroughProperties = (x) -> x;
    MovementProperties stopAtEndProperties = (x) -> x;

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
        this.headings = calculateHeadings(0);
    }

    // TODO try derivatives of the path to get the heading (make sure to not force path continuity)
    // may arise with problems such as transitional correction,
    // and uneven speeds/inaccurate variable t on parameter paths with
    // might have to resolve checking if past a point but that is basically what the system is
    // already doing.
    private double[] calculateHeadings(double headingOffset) {
        double[] headings = new double[this.points.length - 1];

        for (int i = 0; i < this.points.length - 1; i++) {
            double[] point = this.points[i];

            double[] pointHeading;

            int lookAheadIndex = i + Constants.Curve.LOOK_AHEAD_POINTS_FOR_HEADING;
            if (lookAheadIndex >= this.points.length) {
                pointHeading = this.endPoint;
            } else {
                pointHeading = this.points[lookAheadIndex];
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

    public Path setPathProperties(MovementProperties setProperties) {
        moveThroughProperties = setProperties;
        return this;
    }

    public Path setEndPointHoldProperties(MovementProperties setProperties) {
        stopAtEndProperties = setProperties;
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
     * Make the robot smoothly transition from one heading to another along the path.
     */
    public Path setLinearHeadingInterpolation(double startingHeading, double endingHeading) {
        return setLinearHeadingInterpolation(startingHeading, endingHeading, 1.00);
    }

    /**
     * Make the robot smoothly transition from one heading to another along the path.
     *
     * @param finishByPercent The percent of the path to finish the rotation.
     *                       .80 would make the robot turn for 80% of the path.
     */
    public Path setLinearHeadingInterpolation(
        double startingHeading,
        double endingHeading,
        double finishByPercent
    ) {
        headings = new double[this.points.length];

        for (int i = 0; i < this.points.length; i++) {
            int numPointsMinusOne = this.points.length - 1;
            double adjustedFinish = (1 - finishByPercent) * numPointsMinusOne;
            double t = Math.min(1, (double) i / (numPointsMinusOne - adjustedFinish));
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

        Follower.telemetry.addData("i", i);
        Follower.telemetry.update();

        Target.updatePosition();

        double[] point = this.points[i];

        double inchesLeftOnPath = (this.points.length - i) * Constants.Curve.INCHES_PER_POINT;
        if (inchesLeftOnPath < Util.getVectorMagnitude(
            Odometry.xBrakingDistance, Odometry.yBrakingDistance) + 2) {
            startHoldingEndPosition();
            return;
        }

//        double[] pointAhead;
//        if (i + 2 >= this.points.length) {
//            pointAhead = this.endPoint;
//        }
//        else {
//            pointAhead = this.points[i + 1];
//        }

        MovementBuild x = moveThroughProperties.setProperties(
            MovementBuilder.moveThrough(point[0], point[1], this.headings[i])
        )
            .setDriveCorrection(DriveCorrection.fullPower);
        x.isAtGoal = () -> {
            boolean isPastPoint = (Target.y - Odometry.y - Odometry.yBrakingDistance)
                * Target.yDelta
                + (Target.x - Odometry.x - Odometry.xBrakingDistance)
                * Target.xDelta < 0;

            return isPastPoint;
//                if (Vector.getMagnitude(xPower, yPower) < 1) {
//                    break;
//                }
        };
        x.build().waitForMovement(); // TODO build these before running

        this.i += 1;
        if (i >= this.points.length - 1) { // -1
            startHoldingEndPosition();
        }
    }

    private Movement holdEndPosition;

    private void startHoldingEndPosition() {
        holdEndPosition.start();

        holdingEndPosition = true;
    }

    /**
     * Finish building the path. This should be the last method called.
     * If you want to inherit methods
     */
    public Movement build() {
        this.endingHeading = this.headings[headings.length - 1];

        Follower.telemetry.addData("ending Heading", endingHeading);
        Follower.telemetry.update();

        if (stopAtEndOfPath) {
            holdEndPosition = stopAtEndProperties.setProperties(
                MovementBuilder.stopAtPosition(
                endPoint[0],
                endPoint[1],
                endingHeading
            )).setToContinuePowerAfter().build();
        }
        else {
            holdEndPosition = MovementBuilder.moveThrough(
                endPoint[0],
                endPoint[1],
                endingHeading
            ).setToContinuePowerAfter().build();
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

    @NonNull
    @Override
    public Path clone() {
        try {
            return (Path) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}