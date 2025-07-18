package org.firstinspires.ftc.teamcode.blackIce.controller;

import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

import java.util.function.DoubleBinaryOperator;

public class PIDController {
    private final boolean useI, useD;
    
    public final double P, I, D;
    private double previousError, errorIntegral;
    
    public PIDController(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.useI = I != 0;
        this.useD = D != 0;
    }
    
    public double runPID(double target, double current, double deltaTime) {
        double error = target - current;
        double errorRate = useD ? (error - previousError) / deltaTime : 0;
        return runPIDFromError(error, errorRate);
    }
    
    public double runPIDWithCustomErrorChange(double target, double current, double rateOfChange) {
        double error = target - current;
        return runPIDFromError(error, rateOfChange);
    }
    
    public double runPIDFromError(double error, double errorRate) {
        double output = P * error;
        
        if (useI) {
            errorIntegral += error;
            output += I * errorIntegral;
        }
        if (useD) output += D * errorRate;
        
        previousError = error;
        return output;
    }
    
    public Vector runPID(Vector target, Vector current, double deltaTime) {
        return target.map(current, (tar, cur) -> runPID(tar, cur, deltaTime));
    }
    
    public void reset() {
        previousError = 0;
        errorIntegral = 0;
    }
}
