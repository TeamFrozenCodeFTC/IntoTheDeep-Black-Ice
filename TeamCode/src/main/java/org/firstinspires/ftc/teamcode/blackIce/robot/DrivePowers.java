//package org.firstinspires.ftc.teamcode.blackIce.drive;
//
//import org.firstinspires.ftc.teamcode.blackIce.drive.wheelPowers.WheelPowers;
//import org.firstinspires.ftc.teamcode.blackIce.geometry.Vector;
//
///**
// * A set of wheel powers of mecanum drive. Has many useful methods to add, apply, convert, or scale.
// * <pre>
// * FrontLeft, FrontRight
// * BackLeft,  BackRight
// *
// * fl, bl, fr, br
// *
// * fl, fr
// * bl, br
// */
//public class DrivePowers extends WheelPowers<DrivePowers> {
//    public final double frontLeftPower;
//    public final double backLeftPower;
//    public final double frontRightPower;
//    public final double backRightPower;
//    public final double[] components;
//
//    /**
//     * Create a set of wheel powers.
//     * <pre>
//     * FrontLeft, FrontRight
//     * BackLeft,  BackRight
//     *
//     * fl, bl, fr, br
//     *
//     * fl, fr
//     * bl, br
//     */
//    public DrivePowers(double fl, double bl, double fr, double br) {
//        super(4);
//        //this.components = new double[]{fl, bl, fr, br};
//        this.frontLeftPower = fl;
//        this.backLeftPower = bl;
//        this.frontRightPower = fr;
//        this.backRightPower = br;
//    }
//
//    /**
//     * Apply the wheel powers to the drivetrain.
//     */
//    public void applyPowers() {
//        Drive.power(this);
//    }
//
//    /**
//     * Takes a field-relative vector and converts it into wheel powers
//     * that would make the robot move in the direction of the field vector.
//     */
//    @Deprecated
//    public static DrivePowers fromFieldVector(double fieldX, double fieldY) {
//        return fromFieldVector(new Vector(fieldX, fieldY));
//    }
//
//    /**
//     * Takes a field-relative vector and converts it into wheel powers
//     * that would make the robot move in the direction of the field vector.
//     */
//    public static DrivePowers fromFieldVector(Vector fieldVector) {
//        return DrivePowers.fromRobotVector(fieldVector.toRobotVector());
//    }
//
//    /**
//     * Takes a robot-relative vector and converts it into wheel powers.
//     */
//    public static DrivePowers fromRobotVector(Vector robotVector) {
//        double upRightDirection = robotVector.getX() + robotVector.getY();
//        double downLeftDirection = robotVector.getX() - robotVector.getY();
//
//        return new DrivePowers(
//            downLeftDirection, upRightDirection,
//            upRightDirection,  downLeftDirection
//        );
//    }
//
//    public static DrivePowers forward(double power) {
//        return new DrivePowers(power, power, power, power);
//    }
//    public static DrivePowers backward(double power) {
//        return forward(-power);
//    }
//    public static DrivePowers slideLeft(double power) {
//        return new DrivePowers(power, -power, -power, power);
//    }
//    public static DrivePowers slideRight(double power) {
//        return slideLeft(-power);
//    }
//    public static DrivePowers turnClockwise(double power) {
//        return new DrivePowers(power, power, -power, -power);
//    }
//    public static DrivePowers turnCounterclockwise(double power) {
//        return turnClockwise(-power);
//    }
//
//    /**
//     * Add the given powers together,
//     * but only add the powers if they are in the same direction and working together.
//     */
//    public DrivePowers addAlignedPowers(DrivePowers other) {
//        ComponentPairOperator operator = (power1, power2) -> {
//            boolean isAligned = Math.signum(power1) == Math.signum(power2);
//            if (isAligned) {
//                return power1 + power2;
//            } else {
//                return power1;
//            }
//        };
//        return mapPairs(operator, other);
//    }
//
//    /**
//     * Scale the given powers until at least one of them is equal to the given max power.
//     *
//     * <pre><code>
//     * scaleToMax({10, -5, -10, 1}, 1) -> {1, -0.5, -1, 0.1}
//     * scaleToMax({-0.5, 0.25, 0.75, -0.25}, 1) -> {-0.66, 0.33, 1, -0.33}
//     * </code></pre>
//     *
//     * @param newMax The power value this method is scaling to. Must be positive.
//     */
//    public DrivePowers scaleMaxTo(double newMax) {
//        double currentMax = getMaxAbs();
//        if (currentMax == 0) return this;
//        return this.multiply(newMax / currentMax);
//    }
//
//    /**
//     * Scale the given powers down so that
//     * at least one of them is less than or equal to the given max power.
//     *
//     * <pre><code>
//     * {10, -5, -10, 1}.downscaleToMax(1) -> {1, -0.5, -1, 0.1}
//     * {-0.5, 0.25, 0.75, -0.25}.downscaleToMax(1) -> {-0.5, 0.25, 0.75, -0.25}
//     * // Does not change the values because all of them are less than 1.
//     * </code></pre>
//     *
//     * @param max The power value this method is scaling to. Must be positive.
//     */
//    public DrivePowers downscaleMaxTo(double max) {
//        double currentMax = getMaxAbs();
//        if (currentMax <= max) return this;
//        return this.multiply(max / currentMax);
//    }
//
//    /**
//     * Scale the powers so that the maximum value is equal to 1.
//     *
//     * <pre><code>
//     * normalize({2, -4, 2, -1}) -> {0.5, -1, 0.5, -0.25}
//     * normalize({-0.5, 0.25, 0, 0.5}) -> {-1, 0.5, 0, 1}
//     * </code></pre>
//     */
//    public DrivePowers normalize() {
//        return scaleMaxTo(1);
//    }
//
//    /**
//     * Scale the powers so that the maximum value is less than or equal to 1.
//     *
//     * <pre><code>
//     * normalizeDown({2, -4, 2, -1}) -> {0.5, -1, 0.5, -0.25}
//     * normalizeDown({-0.5, 0.25, 0, 0.5}) -> {-0.5, 0.25, 0, 0.5}
//     * </code></pre>
//     *
//     * Different from {@link DrivePowers#normalize} because this method does not upscale the powers
//     * if the max value is less than 1.
//     */
//    public DrivePowers normalizeDown() {
//        return downscaleMaxTo(1);
//    }
//
//    @Override
//    protected DrivePowers fromComponentArray(double[] components) {
//        return new DrivePowers(
//            components[0],
//            components[1],
//            components[2],
//            components[3]
//        );
//    }
//}