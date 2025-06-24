package org.firstinspires.ftc.teamcode.blackIce.paths.pidf;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;

import java.util.function.DoubleBinaryOperator;


/**
 * Represents a PIDF (Proportional-Integral-Derivative-Feedforward) controller.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 */
public class PIDFController {
    private final DoubleBinaryOperator optimizedPID;
    
    public double P, I, D, F;
    private double error, previousError, errorIntegral;
    
    /**
     * Constructs a new PIDFController with the specified gain values.
     *
     * @param P The proportional gain. Determines the controller's response to the current error.
     * @param I The integral gain. Determines the controller's response based on the sum of recent errors.
     * @param D The derivative gain. Determines the controller's response based on the rate of change of error.
     * @param F The feedforward gain. Provides a constant offset to the output, regardless of the error.
     */
    public PIDFController(double P, double I, double D, double F) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
        
        if (I == 0 && D == 0) {
            optimizedPID = (error, deltaTime) -> computeProportional(error);
        } else if (I == 0) {
            optimizedPID =
                (error, deltaTime) -> computeProportional(error) + computeDerivative(error,
                    deltaTime);
        } else if (D == 0) {
            optimizedPID =
                (error, deltaTime) -> computeProportional(error) + computeIntegral(error);
        } else {
            optimizedPID =
                (error, deltaTime) -> computeProportional(error)
                    + computeDerivative(error, deltaTime)
                    + computeIntegral(error);
        }
    }
    
    public static PIDFController createProportionalFeedforward(double P, double F) {
        return new PIDFController(P, 0, 0, F);
    }
    public static PIDFController createProportionalDerivativeFeedforward(double P, double D, double F) {
        return new PIDFController(P, 0, D, F);
    }
    
//    public double run(double target, double current, double deltaTime) {
//        return run(target - current, deltaTime);
//    }
    
    public double run(double target, double current, double deltaTime) {
        error = target - current;
        double output = optimizedPID.applyAsDouble(error, deltaTime);
        previousError = error;
        return output + F * target;
    }
    
    public Vector run(Vector target, Vector current, double deltaTime) {
        return target.map(current, ((tar, cur) -> run(tar, cur, deltaTime)));
    }
    
    private double computeProportional(double error) {
        return error * P;
    }
    
    private double computeDerivative(double error, double deltaTime) {
        double derivative = (error - previousError) / deltaTime;
        return D * derivative;
    }
    
    private double computeIntegral(double error) {
        errorIntegral += error;
        return I * errorIntegral;
    }
    
    public void reset() {
        error = 0;
        previousError = 0;
        errorIntegral = 0;
    }
}
