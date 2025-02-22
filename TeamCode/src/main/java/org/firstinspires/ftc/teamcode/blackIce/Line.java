package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.Arrays;

public class Line extends BezierCurve {
    public Line(double endX, double endY) {
        super(new double[][] {
            {Robot.instance.movement.target.previousX, Robot.instance.movement.target.previousY},
            {endX, endY}
        });
    }
}
