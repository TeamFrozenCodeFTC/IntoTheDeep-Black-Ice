package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.Constants;

import java.util.Arrays;

public class BezierCurve extends Path {
    public BezierCurve(double[][] controlPoints) {
        super(BezierCurve.calculateBezierPoints(controlPoints));
    }

    private static double[][] calculateBezierPoints(double[][] controlPoints) {
        double totalLength = estimateCurveLength(controlPoints, 100);
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
