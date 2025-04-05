package org.firstinspires.ftc.teamcode.blackIce;

public class Vector {
    /**
     * Scales the maximum value in the array to be less than 1
     *
     * <pre><code>
     * normalizeDown({2, 4}) -> {0.5, 1}
     * normalizeDown({0.25, 0.5}) -> {0.25, 0.5}
     * </code></pre>
     */
    public static double[] normalizeDown(double[] a) {
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

    /**
     * Scales the given x and y components until one of them is equal to the given max value.
     *
     * <pre><code>
     * scaleToMax(0.25, 0.5, 1) -> {0.5, 1}
     * </code></pre>
     *
     * @param x The x component.
     * @param y The y component.
     * @param maxValue The positive maximum value to scale to.
     */
    public static double[] scaleToMax(double x, double y, double maxValue) {
        double maxComponent = Math.max(Math.abs(x), Math.abs(y));

        if (maxComponent == 0) {
            return new double[]{0, 0};
        }

        double scale = maxValue / maxComponent;

        return new double[]{x * scale, y * scale};
    }

    private static double getAbsMax(double[] arr) {
        double maxAbs = Math.abs(arr[0]);
        for (int i = 1; i < arr.length; i++) {
            maxAbs = Math.max(maxAbs, Math.abs(arr[i]));
        }

        return maxAbs;
    }

    public static double[] scaleToMax(double[] powers, double maxValue) {
        double maxComponent = getAbsMax(powers);

        if (maxComponent == 0) {
            return new double[]{0, 0, 0, 0};
        }

        double scale = maxValue / maxComponent;

        return new double[]{
            powers[0] * scale,
            powers[1] * scale,
            powers[2] * scale,
            powers[3] * scale,
        };
    }


    /**
     * Downscales the given x and y components until
     * one of them is equal or less than the given max value
     * (works with negative and positive numbers).
     *
     * <pre><code>
     * downscaleToMax(10, 5, 1) -> {1, 0.5}
     * </code></pre>
     *
     * @param x The x component.
     * @param y The y component.
     * @param maxValue A positive maximum value to downscale to.
     */
    public static double[] downscaleToMax(double x, double y, double maxValue) {
        double maxComponent = Math.max(Math.abs(x), Math.abs(y));

        if (maxComponent <= maxValue || maxComponent == 0) {
            return new double[]{x, y};
        }

        double scale = maxValue / maxComponent;

        return new double[]{x * scale, y * scale};
    }

    public static double getMagnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    public static double[] setMagnitude(double x, double y, double newMagnitude) {
        double magnitude = getMagnitude(x, y);

        return new double[]{
            x / magnitude * newMagnitude,
            y / magnitude * newMagnitude
        };
    }

    public static double[] setMagnitudeToOne(double x, double y) {
        return setMagnitude(x, y, 1);
    }


}
