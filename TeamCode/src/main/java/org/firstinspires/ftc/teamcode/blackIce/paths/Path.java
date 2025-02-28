package org.firstinspires.ftc.teamcode.blackIce.paths;

public class Path {
    double[][] points;

    double constantHeading = 0;
    double headingOffset = 0;
    boolean isConstantHeading = false;

    double endingHeading;
    double[] endPoint;

    /**
     * Create a path of points the robot follows.
     */
    public Path(double[][] points) {
        this.points = points;

        this.endPoint = points[points.length - 1];

        if (isConstantHeading) {
            endingHeading = constantHeading;
        } else {
            endingHeading = Math.toDegrees(
                Math.atan2(endPoint[1] - points[points.length - 2][1],
                    endPoint[0] - points[points.length - 2][0])
            ) + headingOffset;
        }
    }

    /**
     * Make the robot have a constant heading along the path.
     */
    public Path setConstantHeading(double newConstantHeading) {
        isConstantHeading = true;
        constantHeading = newConstantHeading;
        return this;
    }

    /**
     * Make the robot have a offset heading along the path.
     */
    public Path setHeadingOffset(double newHeadingOffset) {
        isConstantHeading = false;
        headingOffset = newHeadingOffset;
        return this;
    }
}