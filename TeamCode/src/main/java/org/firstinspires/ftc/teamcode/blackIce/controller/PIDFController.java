package org.firstinspires.ftc.teamcode.blackIce.controller;


/**
 * Represents a PIDF (Proportional-Integral-Derivative-Feedforward) controller.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 */
public class PIDFController extends PIDController {
    @FunctionalInterface
    public interface Feedforward {
        double compute(double target);
    }
    
    public final Feedforward F;
    
    public PIDFController(double P, double I, double D, double F) {
        this(P, I, D, target -> target * F);
    }
    
    public PIDFController(double P, double I, double D, Feedforward F) {
        super(P, I, D);
        this.F = F;
    }
    
    public double runPIDF(double target, double current, double deltaTime) {
        return super.runPID(target, current, deltaTime) + F.compute(target);
    }
}
