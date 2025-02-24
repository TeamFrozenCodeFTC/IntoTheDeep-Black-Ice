package org.firstinspires.ftc.teamcode.blackIce;

public class BrakingDisplacement {
    double a;
    double b;
    double c;

    public BrakingDisplacement(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public double predict(double x) {
        return Math.signum(x) * a * Math.pow(x, 2) + b * x + c;
    }
}
