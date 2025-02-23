package org.firstinspires.ftc.teamcode.blackIce.paths;

import org.firstinspires.ftc.teamcode.blackIce.Target;

public class Line extends BezierCurve {
    public Line(double endX, double endY) {
        super(new double[][] {
            {Target.previousX, Target.previousY},
            {endX, endY}
        });
    }
}
