package org.firstinspires.ftc.teamcode.blackIce.math.kinematics;

public class QuadraticBrakingModel implements BrakingDistanceModel {
    final double deceleration;

    public QuadraticBrakingModel(double deceleration) {
        this.deceleration = deceleration;
    }

    /**
     * Predicts directional braking distance (aka braking distance can be negative).
     * <pre>
     * f(x) = a·x·abs(x)
     * </pre>
     */
    public double getStoppingDistanceWithVelocity(double x) {
        return Kinematics.getStoppingDistance(x, deceleration);
    }

    public double getTargetVelocityToStopAtDistance(double directionalDistance) {
        return Kinematics.getCurrentVelocityToStopAtPositionWithDeceleration(directionalDistance, deceleration);
    }
}