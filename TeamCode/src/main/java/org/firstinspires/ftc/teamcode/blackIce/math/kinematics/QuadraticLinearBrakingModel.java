package org.firstinspires.ftc.teamcode.blackIce.math.kinematics;

public class QuadraticLinearBrakingModel implements BrakingDistanceModel {
    final double a;
    final double b;

    public QuadraticLinearBrakingModel(double a, double b) {
        this.a = a;
        this.b = b;
    }

    /**
     * Predicts directional braking distance (aka braking distance can be negative).
     * <pre>
     * f(x) = a·x·abs(x) + b·x
     * </pre>
     */
    public double getStoppingDistanceWithVelocity(double x) {
        return a * x * Math.abs(x) + (b * x);
    }

//    /**
//     * Updates a and b based on the prediction error.
//     * Call this after a braking event finishes and you measure actual distance.
//     */
//    public void update(double v, double actualDistance) {
//        double predicted = (a * v * Math.abs(v)) + (b * v);
//        double error = predicted - actualDistance;
//
//        double ax = v * Math.abs(v);
//        double bx = v;
//
//        a -= 1e-8 * error * ax;
//        b -= 5e-7 * error * bx;
//    }

    public double getTargetVelocityToStopAtDistance(double directionalDistance) {
        double discriminant = Math.abs(b * b + 4 * a * directionalDistance);
        double sqrtD = Math.signum(directionalDistance) * Math.sqrt(discriminant);
        return (-b + sqrtD) / (2 * a);
    }

//    public double getTargetVelocityToReachVelocityAtDistance(double directionalDistance,
//                                                             double targetVelocity) {
//        double discriminant = Math.abs(b * b + 4 * a * directionalDistance);
//        double sqrtD = Math.signum(directionalDistance) * Math.sqrt(discriminant);
//        return (-b + sqrtD) / (2 * a);
//    }
}