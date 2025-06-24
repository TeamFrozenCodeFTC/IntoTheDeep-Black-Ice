package org.firstinspires.ftc.teamcode.blackIce.paths.calculators;

public interface TranslationalVectorCalculator extends VectorCalculatorComponent {
    TranslationalVectorCalculator toClosestPose = (context) -> context.closestPoint.getPoint()
        .subtract(context.motionState.getPredictedStoppedPosition());
//    TranslationalVectorCalculator toEndPose = (context) -> context.endPoint.getPoint()
//        .subtract(context.motionState.getPredictedStoppedPosition());
}
