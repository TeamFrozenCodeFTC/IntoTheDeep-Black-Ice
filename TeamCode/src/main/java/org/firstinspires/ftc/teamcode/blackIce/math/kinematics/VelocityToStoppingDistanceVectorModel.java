package org.firstinspires.ftc.teamcode.blackIce.math.kinematics;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

public class VelocityToStoppingDistanceVectorModel {
    final BrakingDistanceModel forwardModel;
    final BrakingDistanceModel lateralModel;

    public VelocityToStoppingDistanceVectorModel(
        BrakingDistanceModel forwardModel,
        BrakingDistanceModel lateralModel
    ) {
        this.forwardModel = forwardModel;
        this.lateralModel = lateralModel;
    }

    /**
     * Predicts directional braking distance (aka braking distance can be negative).
     * <pre>
     * f(x) = a·x·abs(x) + b·x
     * </pre>
     */
    public Vector getStoppingDistanceWithVelocity(Vector velocity) {
        return new Vector(
            forwardModel.getStoppingDistanceWithVelocity(velocity.getX()),
            lateralModel.getStoppingDistanceWithVelocity(velocity.getY())
        );
    }

    public Vector getTargetVelocityToStopAtDistance(
        Vector directionalDistance
    ) {
        return new Vector(
            forwardModel.getTargetVelocityToStopAtDistance(directionalDistance.getX()),
            lateralModel.getTargetVelocityToStopAtDistance(directionalDistance.getY())
        );
    }
}