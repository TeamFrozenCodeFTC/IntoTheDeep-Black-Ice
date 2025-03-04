package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.Constants;

import java.util.Arrays;

/**
 * A Bezier Curve <a href="https://en.wikipedia.org/wiki/B%C3%A9zier_curve">...</a> is a
 * smooth curve made with control points.
 */
public class BezierCurve extends Path {
    final double totalLength;

    /**
     * Create a {@link BezierCurve} from control points.
     *
     * @param controlPoints Example: {@code new double[][] {{1, 2}, {2, 3}, ...}}
     */
    public BezierCurve(double[][] controlPoints) {
        this(controlPoints, BezierCurve.estimateCurveLength(controlPoints, 1000));
    }

    private BezierCurve(double[][] controlPoints, double totalLength) {
        super(BezierCurve.calculateBezierPoints(controlPoints, totalLength));
        this.totalLength = totalLength;
    }

    private static double[][] calculateBezierPoints(double[][] controlPoints, double totalLength) {
        int numPoints = Math.max(1, (int) (totalLength / Constants.Curve.INCHES_PER_POINT));

        double[][] curvePoints = new double[numPoints + 1][2];
        for (int i = 0; i <= numPoints; i++) {
            double t = (double) i / numPoints;
            curvePoints[i] = deCasteljau(controlPoints, t);
        }

        return curvePoints;
    }

    private static double[] deCasteljau(double[][] points, double t) {
        int n = points.length;
        double[][] tempPoints = Arrays.copyOf(points, n);

        for (int r = 1; r < n; r++) {
            for (int i = 0; i < n - r; i++) {
                tempPoints[i] = new double[]{
                    (1 - t) * tempPoints[i][0] + t * tempPoints[i + 1][0],
                    (1 - t) * tempPoints[i][1] + t * tempPoints[i + 1][1]
                };
            }
        }

        return tempPoints[0];
    }

    private static double estimateCurveLength(double[][] controlPoints, int samples) {
        double length = 0;
        double[] prev = deCasteljau(controlPoints, 0);

        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            double[] curr = deCasteljau(controlPoints, t);
            length += Math.hypot(curr[0] - prev[0], curr[1] - prev[1]);
            prev = curr;
        }

        return length;
    }
}
