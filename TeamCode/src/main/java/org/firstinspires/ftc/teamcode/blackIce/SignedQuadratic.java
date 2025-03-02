package org.firstinspires.ftc.teamcode.blackIce;

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

    /**
     * Predicts directional braking distance (aka braking distance can be negative).
     * <pre>
     * f(x) = a·x·abs(x) + b·x + c·sgn(x)
     * </pre>
     */
    public double predict(double x) {
        return (a * x * Math.abs(x)) + (b * x) + (Math.signum(x) * c);
    }
}
