package org.firstinspires.ftc.teamcode.blackIce.drive;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.blackIce.Vector;

/**
 * A set of wheel powers. Has many useful methods to add, apply, convert, or scale.
 */
public class DrivePowers {
    public double frontLeftPower;
    public double backLeftPower;
    public double frontRightPower;
    public double backRightPower;

    /**
     * Create a set of wheel powers.
     * <pre>
     * FrontLeft, FrontRight
     * BackLeft,  BackRight
     *
     * fl, fr
     * bl, br
     */
    public DrivePowers(double fl, double bl, double fr, double br) {
        // want to change this to fl, fr, ... but would have to change .slideLeft(), .turn(), etc.
        this.frontLeftPower = fl;
        this.backLeftPower = bl;
        this.frontRightPower = fr;
        this.backRightPower = br;
    }

    /**
     * Apply the wheel powers to the drivetrain.
     */
    public void applyPowers() {
        Drive.power(this);
    }

    /**
     * Takes a field-relative vector and converts it into wheel powers
     * that would make the robot move in the direction of the field vector.
     */
    public static DrivePowers fromFieldVector(double fieldX, double fieldY) {
        return new Vector(fieldX, fieldY).fieldVectorToLocalWheelPowers();
    }

    public static DrivePowers forward(double power) {
        return new DrivePowers(power, power, power, power);
    }

    public static DrivePowers backward(double power) {
        return forward(-power);
    }

    public static  DrivePowers slideLeft(double power) {
        return new DrivePowers(power, -power, -power, power);
    }

    public static DrivePowers slideRight(double power) {
        return slideLeft(-power);
    }

    public static DrivePowers turnClockwise(double power) {
        return new DrivePowers(power, power, -power, -power);
    }

    public static DrivePowers turnCounterclockwise(double power) {
        return turnClockwise(-power);
    }

    /**
     * Add the given powers together.
     */
    public DrivePowers add(DrivePowers other) {
        return new DrivePowers(
            this.frontLeftPower + other.frontLeftPower,
            this.backLeftPower + other.backLeftPower,
            this.frontRightPower + other.frontRightPower,
            this.backRightPower + other.backRightPower
        );
    }

    /**
     * Add the given powers together,
     * but only adds the powers if they are in the same direction / working together.
     */
    public DrivePowers addAlignedPowers(DrivePowers other) {
        // This method is mutable which isn't consistent with the rest of the class
        // Probably not the best thing and this.powers should be final and immutable
        // But it saves code and has negligent performance of not assigning more local variables

        if (Math.signum(this.frontLeftPower) == Math.signum(other.frontLeftPower)) {
            this.frontLeftPower += other.frontLeftPower;
        }
        if (Math.signum(this.frontRightPower) == Math.signum(other.frontRightPower)) {
            this.frontRightPower += other.frontRightPower;
        }
        if (Math.signum(this.backLeftPower) == Math.signum(other.backLeftPower)) {
            this.backLeftPower += other.backLeftPower;
        }
        if (Math.signum(this.backRightPower) == Math.signum(other.backRightPower)) {
            this.backRightPower += other.backRightPower;
        }

        return new DrivePowers(
            this.frontLeftPower,
            this.backLeftPower,
            this.frontRightPower,
            this.backRightPower
        );
    }

    /**
     * Multiply the powers by the given scale factor.
     */
    public DrivePowers scale(double scaleFactor) {
        return new DrivePowers(
            frontLeftPower * scaleFactor,
            backLeftPower * scaleFactor,
            frontRightPower * scaleFactor,
            backRightPower * scaleFactor
        );
    }

    /**
     * Scales the given powers until at least one of them is equal to the given max power.
     *
     * <pre><code>
     * scaleToMax({10, -5, -10, 1}, 1) -> {1, -0.5, -1, 0.1}
     * scaleToMax({-0.5, 0.25, 0.75, -0.25}, 1) -> {-0.66, 0.33, 1, -0.33}
     * </code></pre>
     *
     * @param newMax The power value this method is scaling to. Must be positive.
     */
    public DrivePowers scaleMaxTo(double newMax) {
        double currentMax = getMaxAbs();
        if (currentMax == 0) return this;
        return this.scale(newMax / currentMax);
    }

    /**
     * Scales the given powers until
     * at least one of them is less than or equal to the given max power.
     *
     * <pre><code>
     * downscaleToMax({10, -5, -10, 1}, 1) -> {1, -0.5, -1, 0.1}
     * downscaleToMax({-0.5, 0.25, 0.75, -0.25}, 1) -> {-0.5, 0.25, 0.75, -0.25}
     * // Does not change the values because all of them are less than 1.
     * </code></pre>
     *
     * @param max The power value this method is scaling to. Must be positive.
     */
    public DrivePowers downscaleMaxTo(double max) {
        double currentMax = getMaxAbs();
        if (currentMax <= max) return this;
        return this.scale(max / currentMax);
    }

    /**
     * Scale the powers so that the maximum value is equal to 1.
     *
     * <pre><code>
     * normalize({2, -4, 2, -1}) -> {0.5, -1, 0.5, -0.25}
     * normalize({-0.5, 0.25, 0, 0.5}) -> {-1, 0.5, 0, 1}
     * </code></pre>
     */
    public DrivePowers normalize() {
        return scaleMaxTo(1);
    }

    /**
     * Scale the powers so that the maximum value is less than or equal to 1.
     *
     * <pre><code>
     * normalizeDown({2, -4, 2, -1}) -> {0.5, -1, 0.5, -0.25}
     * normalizeDown({-0.5, 0.25, 0, 0.5}) -> {-0.5, 0.25, 0, 0.5}
     * </code></pre>
     *
     * Different from {@link DrivePowers#normalize} because this method does not upscale the powers
     * if the max value is less than 1.
     */
    public DrivePowers normalizeDown() {
        return downscaleMaxTo(1);
    }

    private double getMaxAbs() {
        return Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        );
    }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        return String.format("FL: %.2f, FR: %.2f,\nBL: %.2f, BR: %.2f",
            frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }
}