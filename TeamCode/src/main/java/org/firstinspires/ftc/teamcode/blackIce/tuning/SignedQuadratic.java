package org.firstinspires.ftc.teamcode.blackIce.tuning;

/**
 * A piecewise, signed quadratic that can output both positive and negative numbers.
 * Similar to the graph of {@code x * |x|} or {@code x * abs(x)}.
 * <p>
 * Used to predict directional braking distance (aka braking distance can be negative).
 * <pre>
 * f(x) = a·x·abs(x) + b·x + c·sgn(x)
 * </pre>
 */
public class SignedQuadratic {
    double a;
    double b;
    double c;

    public SignedQuadratic(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    // pedro path uses zero power deceleration and then multiples it by a constant
    // braking deceleration -> predicting distance

    // TODO progressive braking distance
    // TODO remove c term

    // needs single x term to account for constant braking force coefficient

    /**
     * Predicts directional braking distance (aka braking distance can be negative).
     * <pre>
     * f(x) = a·x·abs(x) + b·x + c·sgn(x)
     * </pre>
     */
    public double predict(double n) {
        return (a * n * Math.abs(n)) + (b * n) + (Math.signum(n) * c); // c term?
    }
}
