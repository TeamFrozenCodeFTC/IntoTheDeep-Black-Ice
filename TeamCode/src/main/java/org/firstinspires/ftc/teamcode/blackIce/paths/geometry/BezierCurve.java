package org.firstinspires.ftc.teamcode.blackIce.paths.geometry;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

/**
 * This is the BezierCurve class. This class handles the creation of Bezier curves, which are used
 * as the basis of the path for the Path class. Bezier curves are parametric curves defined by a set
 * of control points. So, they take in one input, t, that ranges from [0, 1] and that returns a point
 * on the curve. You can read more on Bezier curves here:
 * <a href="https://en.wikipedia.org/wiki/B">Bézier Curve</a>
 */
public class BezierCurve implements PathGeometry {
    private final Vector[] controlPoints;
    private final double length;
    private final PathPoint endPathPoint;

//    public BezierCurve(double[][] controlPoints) {
//        Vector[] vectorControlPoints = new Vector[controlPoints.length];
//        for (int i = 0; i < controlPoints.length; i++) {
//            vectorControlPoints[i] = new Vector(controlPoints[i][0], controlPoints[i][1]);
//        }
//        this(vectorControlPoints);
//    }
    
    public BezierCurve(Vector... controlPoints) {
        this.controlPoints = controlPoints;
        this.length = calculateCurveLength(1000);
        this.endPathPoint = new PathPoint(1, calculateFirstDerivative(1),
            this.controlPoints[this.controlPoints.length - 1]);
    }

    @Override
    public double length() {
        return length;
    }
    
    @Override
    public PathPoint getEndPathPoint() {
        return endPathPoint;
    }

    public Vector computePointAt(double t) {
        int n = controlPoints.length - 1;
        Vector point = new Vector(0, 0);

        for (int i = 0; i <= n; i++) {
            double bernstein = binomial(n, i) * Math.pow(1 - t, n - i) * Math.pow(t, i);
            point = point.plus(controlPoints[i].times(bernstein));
        }

        return point;
    }

    private int binomial(int n, int k) {
        int result = 1;
        for (int i = 1; i <= k; i++) {
            result *= (n - (k - i));
            result /= i;
        }
        return result;
    }

    /**
     * This is equal to the tangent vector of the curve at t.
     */
    public Vector calculateFirstDerivative(double t) {
        int n = controlPoints.length - 1;
        Vector derivative = new Vector(0, 0);

        for (int i = 0; i < n; i++) {
            double bernstein = binomial(n - 1, i) * Math.pow(1 - t, n - 1 - i) * Math.pow(t, i);
            Vector diff = controlPoints[i + 1].minus(controlPoints[i]);
            derivative = derivative.plus(diff.times(bernstein));
        }

        return derivative.times(n);
    }
    
    public Vector calculateSecondDerivative(double t) {
        int n = controlPoints.length - 1;
        Vector secondDerivative = new Vector(0, 0);
        
        for (int i = 0; i <= n - 2; i++) {
            double bernstein = binomial(n - 2, i) * Math.pow(1 - t, n - 2 - i) * Math.pow(t, i);
            Vector diff = controlPoints[i + 2]
                .minus(controlPoints[i + 1].times(2))
                .plus(controlPoints[i]);
            secondDerivative = secondDerivative.plus(diff.times(bernstein));
        }
        
        return secondDerivative.times(n * (n - 1));
    }
    
    public double calculateCurveLength(int numSamples) {
        double length = 0;
        Vector prevPoint = computePointAt(0);

        for (int i = 1; i <= numSamples; i++) {
            double t = (double) i / numSamples;
            Vector point = computePointAt(t);
            length += Math.sqrt(point.minus(prevPoint).lengthSquared());
            prevPoint = point;
        }

        return length;
    }
    
    @Override
    public PathPoint computeClosestPathPointTo(Vector target, double startingGuess) {
//        /**
//         * Predict the next t value based how much velocity the robot
//         * is going along the direction of the tangent line of the path.
//         */
//        double predictNextT(Vector tangentVector) {
//            double percentAlignedOnPath = calculateVelocityAlignmentWithTangent(tangentVector);
//            double velocityAlongPath = motionState.velocityMagnitude * percentAlignedOnPath;
//            return currentSegmentT + (velocityAlongPath / length) * motionState.deltaTime;
//        } // last tangent vector
        double t = startingGuess;
        double epsilon = 0.01; // was 0.05
        double epsilonSquared = epsilon * epsilon;
        int maxIterations = 100;
        int iteration = 0;

        Vector firstDerivative;
        Vector bezierPoint;

        do {
            bezierPoint = computePointAt(t);
            firstDerivative = calculateFirstDerivative(t);

            Vector errorVector = target.minus(bezierPoint);

            Vector secondDerivative = calculateSecondDerivative(t);
            
            double numerator = errorVector.dotProduct(firstDerivative);
            double denominator = firstDerivative.lengthSquared() + errorVector.dotProduct(secondDerivative);
            
            if (Math.abs(denominator) < 1e-6) break; // Prevent divide-by-near-zero

            double deltaT = numerator / denominator;

            t = PathGeometry.clampT(t + deltaT);
            if (Math.abs(deltaT) < 1e-6) {
                break;
            }

            iteration++;
        } while (iteration <= maxIterations);

        Vector tangentVector = firstDerivative.normalized();
        return new PathPoint(t, tangentVector, bezierPoint);
    }
}