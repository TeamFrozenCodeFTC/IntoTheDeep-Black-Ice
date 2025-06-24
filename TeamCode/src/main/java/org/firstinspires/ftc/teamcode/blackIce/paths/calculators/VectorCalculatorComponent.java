package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

@FunctionalInterface
public interface VectorCalculatorComponent {
    // return a robot-relative vector
    Vector computeTargetVector(PathFollowContext context);
}
