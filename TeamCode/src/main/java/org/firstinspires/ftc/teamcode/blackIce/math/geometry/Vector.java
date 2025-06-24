package org.firstinspires.ftc.teamcode.blackIce.math.geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.blackIce.robot.OperableComponents;

/**
 * Immutable 2D vector class that represents drive vectors and robot positions.
 * Points are converted to vectors for math operations for the path following.
 * Inputs to methods use the Point or Pose class.
 * This class is used for both field-relative and robot-relative vectors.
 */
public class Vector extends OperableComponents<Vector> {
    /** The unit vector that moves the robot forward when at 0 heading. */
    public static Vector FORWARD = new Vector(1,0);
    
    /** The unit vector that moves the robot forward when at 0 heading. */
    public static Vector BACKWARD = new Vector(-1,0);
    
    /** The unit vector that strafes the robot to the left when at 0 heading. */
    public static Vector LEFT = new Vector(0,1);
    
    /** The unit vector that strafes the robot to the right when at 0 heading. */
    public static Vector RIGHT = new Vector(0,-1);
    
    
    public Vector(double x, double y) {
        super(new double[]{x, y});
    }
    
    /**
     * Creates a vector with the same x and y values from a scalar.
     * <pre><code>
     * Vector.fromScalar(5) -> new Vector(5, 5)
     * </code></pre>
     */
    public static Vector fromScalar(double xAndYComponent) {
        return new Vector(xAndYComponent, xAndYComponent);
    }
    
    public double getX() {
        return this.components[0];
    }

    public double getY() {
        return this.components[1];
    }

    public Vector withX(double x) {
        return new Vector(x, this.getY());
    }

    public Vector withY(double y) {
        return new Vector(this.getX(), y);
    }
    
    public boolean equals(Vector obj) {
        if (this == obj) return true;

        return this.getX() == obj.getX() && this.getY() == obj.getY();
    }

    public static Vector fromMagnitude(double magnitude, double angleRadians) {
        return new Vector(
            magnitude * Math.cos(angleRadians),
            magnitude * Math.sin(angleRadians)
        );
    }
    
    @SuppressLint("DefaultLocale")
    @NonNull
    public String toString() {
        return String.format("Vector{x=%.2f, y=%.2f}", getX(), getY());
    }

    public double dot(Vector other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }

    /**
     * Applies the rotation to this vector by the given angle in radians.
     * Positive angles are counterclockwise, so this rotates the vector counterclockwise.
     */
    public Vector rotatedBy(double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        return new Vector(getX() * cos - getY() * sin, getX() * sin + getY() * cos);
    }

    /**
     * Computes forward and lateral distances from the robot to a target point.
     * @param robotPosition current robot position (x, y)
     * @param robotHeading current robot heading (in radians)
     * @param targetPoint target (x, y) point
     * @return A Vector where:
     *         x = forward distance,
     *         y = lateral (strafe) distance
     */
    public static Vector calculateForwardAndLateralOffset(
        Vector robotPosition,
        double robotHeading,
        Vector targetPoint
    ) {
        Vector toTarget = targetPoint.subtract(robotPosition);
        return toTarget.rotatedBy(-robotHeading);
    }

    public double computeMagnitude() {
        return Math.hypot(getX(), getY());
    }

    public double lengthSquared() {
        return getX() * getX() + getY() * getY();
    }

    public double distanceTo(Vector point) {
        double dx = point.getX() - this.getX();
        double dy = point.getY() - this.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public Vector withMagnitude(double newMagnitude) {
        double magnitude = computeMagnitude();
        return this.divide(magnitude).times(newMagnitude);
    }

    public Vector withMaxMagnitude(double maxMagnitude) {
        double magnitude = computeMagnitude();
        if (magnitude <= maxMagnitude) {
            return this;
        }
        return this.divide(magnitude).times(maxMagnitude);
    }
    
    public Vector withMinMagnitude(double minMagnitude) {
        double magnitude = computeMagnitude();
        if (magnitude > minMagnitude) {
            return this;
        }
        return this.divide(magnitude).times(minMagnitude);
    }

    public Vector normalized() {
        return withMagnitude(1);
    }

    /**
     * Turn a field-relative vector into a robot-relative vector.
     */
    public Vector toRobotVector(double heading) {
        return this.rotatedBy(-heading);
    }
    /**
     * Turn a robot-relative vector into a field-relative vector.
     */
    public Vector toFieldVector(double heading) {
        return this.rotatedBy(heading);
    }

    public double getAngleToLookAt(Vector point) {
        Vector direction = this.subtract(point);
        return Math.atan2(direction.getY(), direction.getX());
    }

    /**
     * Returns the angle of this vector in degrees, counterclockwise from the positive X-axis.
     */
    public double calculateAngle() {
        return Math.toDegrees(Math.atan2(getY(), getX()));
    }

    public double calculateRadians() {
        return Math.atan2(getY(), getX());
    }
    
    @Override
    protected Vector fromComponentArray(double[] components) {
        return new Vector(components[0], components[1]);
    }
}
