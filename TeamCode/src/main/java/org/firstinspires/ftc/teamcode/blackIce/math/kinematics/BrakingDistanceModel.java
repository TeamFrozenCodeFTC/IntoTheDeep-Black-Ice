package org.firstinspires.ftc.teamcode.blackIce.math.kinematics;

public interface BrakingDistanceModel {
    double getStoppingDistanceWithVelocity(double velocity);
    double getTargetVelocityToStopAtDistance(double directionalDistance);
}
