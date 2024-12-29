package org.firstinspires.ftc.teamcode.util;

public class Util {
    public static double clampPower(double x) {
        return Math.max(-1, Math.min(x, 1));
    }

    public static double[] normalize(double[] a) {
        double maxPower = 1.0;
        for (double value : a) {
            maxPower = Math.max(maxPower, Math.abs(value));
        }

        return new double[]{
                a[0] / maxPower,
                a[1] / maxPower,
                a[2] / maxPower,
                a[3] / maxPower
        };
    }

    public static double simplifyAngle(double angle) {
        angle = (angle + 180) % 360;  // Shift range to 0-360
        if (angle < 0) angle += 360; // Handle negative modulo result
        return angle - 180;          // Shift range back to -180 to 180
    }
}
