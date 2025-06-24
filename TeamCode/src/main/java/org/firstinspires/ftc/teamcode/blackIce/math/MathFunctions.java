package org.firstinspires.ftc.teamcode.blackIce.math;

public class MathFunctions {
    public static double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        }
        return Math.min(value, max);
    }

    public static double clamp0To1(double value) {
        return clamp(value, 0, 1);
    }
    
    /**
     * Returns numerator / denominator unless denominator is too close to zero.
     * In that case, returns zero (or a default fallback).
     */
    public static double safeDivide(double numerator, double denominator) {
        return safeDivide(numerator, denominator, 0.0);
    } // TODO use this method
    
    /**
     * Safe divide with a fallback value.
     */
    public static double safeDivide(double numerator, double denominator, double fallback) {
        final double EPSILON = 1e-6;  // tolerance for "zero"
        if (Math.abs(denominator) > EPSILON) {
            return numerator / denominator;
        } else {
            return fallback;
        }
    }
}
