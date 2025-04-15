package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.blackIce.drive.DrivePowers;

public final class Vector { // FieldVector?
    public double x;
    public double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static double getMagnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    public double getMagnitude() {
        return Vector.getMagnitude(x, y);
    }

    public Vector setMagnitude(double x, double y, double newMagnitude) {
        double magnitude = getMagnitude(x, y);

        return new Vector(
            x / magnitude * newMagnitude,
            y / magnitude * newMagnitude
        );
    }

    public Vector normalized(double x, double y) {
        return setMagnitude(x, y, 1);
    }

    /**
     * Takes a field-relative vector and converts it into wheel powers
     * that would make the robot move in the direction of the field vector.
     * <p>
     * Does this by rotating the vector relative to the robot heading where it adds up the lateral
     * and forwards/backwards.
     */
    public DrivePowers fieldVectorToLocalWheelPowers() {
        return this.toRobotVector().robotVectorToLocalWheelPowers();
    }

    public Vector toRobotVector() {
        // positive heading is counterclockwise
        double localForwards = (this.x * Target.headingCos + this.y * Target.headingSin); // clockwise rotation
        double localSlide = (-this.x * Target.headingSin + this.y * Target.headingCos);

        return new Vector(localForwards, localSlide);
    }

    public Vector toFieldVector() {
        // Positive heading is counterclockwise
        double fieldX = (this.x * Target.headingCos - this.y * Target.headingSin); // Counterclockwise rotation
        double fieldY = (this.x * Target.headingSin + this.y * Target.headingCos);

        return new Vector(fieldX, fieldY);
    }

    public DrivePowers robotVectorToLocalWheelPowers() {
        double upRightDirection = this.x + this.y;
        double downLeftDirection = this.x - this.y;

        // TODO Util.normalize was here
        return new DrivePowers(
            downLeftDirection, upRightDirection,
            upRightDirection,  downLeftDirection
        );
    }
}
