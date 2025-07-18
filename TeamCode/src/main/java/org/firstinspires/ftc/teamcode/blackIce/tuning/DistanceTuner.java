package org.firstinspires.ftc.teamcode.blackIce.tuning;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.follower.Follower;
import org.firstinspires.ftc.teamcode.blackIce.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackIce.util.Logger;

import java.util.ArrayList;
import java.util.List;
// TODO make tele-op not reset slide encoders

/**
 * Calculates various braking distances at different velocities.
 * It then uses a quadratic regression algorithm to derive a formula that predicts braking distance
 * based on the robot's current velocity.
 */
public abstract class DistanceTuner extends LinearOpMode {
    /**
     * The amount of different velocities the robot calculates the braking distance at.
     * The braking distance predictions will be more accurate the more points.
     */
    public static int POINTS = 10;
    public static double kBRAKE = 0.015;
    public static double MAX_POWER_REVERSAL = 0.3;

    @SuppressLint("DefaultLocale")
    public List<double[]> run(double heading) {
        Follower follower = new Follower(this);
        
        waitForStart();
        
        List<double[]> data = new ArrayList<>();

        for (int i = 0; i <= POINTS; i++) {
            double percentageDone = (double) i / POINTS;
            double power = (1 - percentageDone);

            double target;
            if (i % 2 == 0) {
                target = 36;
                while (follower.motionState.position.getX() < target) {
                    follower.update();
                    follower.drivetrain.followVector(Vector.FORWARD.rotateCounterclockwiseBy(Math.toRadians(heading)).times(power+0.2), 0);
                }
            }
            else {
                target = 0;
                while (follower.motionState.position.getX() > target) {
                    follower.update();
                    follower.drivetrain.followVector(Vector.BACKWARD.rotateCounterclockwiseBy(Math.toRadians(heading)).times(power+0.2), 0);
                }
            }

            follower.update();
            Vector startingPos = follower.motionState.position
                .plus(follower.motionState.velocity.times(follower.motionState.deltaTime));
            double maxVelocity = Math.abs(follower.motionState.robotRelativeVelocity.getX());
            double sign = Math.signum(follower.motionState.robotRelativeVelocity.getX());
            
            // targetAccel * k, zero power float,
            // decelerating with targetVelocity*k - (targetAcceleration+40)*kA
            
            // 40 * kA = -targetVelocity*kV
            
//            while (follower.motionState.velocity.computeMagnitude() > 0.005) {
            while (Math.signum(follower.motionState.velocity.getX()) == sign) {
                follower.update();
                follower.drivetrain.followVector(new Vector(-0.0001 * sign, 0), 0);
            }
            
            // -0.0001 not quadratic or linear, roughly equal to zero power brake mode, 0
            // 0.001x^2+0.07x. prob why (x-brakingDis) works but max can be -0.0001 only if it
            // isnt split into tangent and translational cause otherwise translational will eat
            // it up. just set to -0.3 as the max
            
            // -0.0001 y = 0.001x^2 + 0.08x
            
            // -0.3 ? y = 0.00142x^2 + 0.03178x // 0.00142xx + 0.03x
            // -velocity * k, mostly linear?
            // -abs(velocity) * velocity * k?
            
            follower.update();
            Vector newDistance = follower.motionState.position;

            double brakingDistance = newDistance.minus(startingPos).computeMagnitude();

            data.add(new double[]{maxVelocity, brakingDistance});
            telemetry.addData(
                Integer.toString(i), stringify(new double[]{maxVelocity, brakingDistance}));
            
            // Correct heading
            while (Math.abs(follower.motionState.heading) > 0.1
                || new Vector(target, 0).minus(follower.motionState.position).computeMagnitude() > 0.25
                || follower.motionState.velocity.computeMagnitude() > 0.05
            ) {
                follower.update();
                follower.drivetrain.followVector(
                    new Vector(target, 0).minus(follower.motionState.position).times(0.25),
                    Math.toRadians(heading)-follower.motionState.heading * 1);
            }
        }

        double[] coefficients = QuadraticRegression.quadraticFit(data);
        Logger.info("data", data.toString());
        Logger.info(String.format("y = %.5fx^2 + %.5fx", coefficients[1], coefficients[0]));
        
        telemetry.addData(
            "Final Equation",
            String.format("y = %.5fx^2 + %.5fx", coefficients[1], coefficients[0]));
        telemetry.addData(
            "Plug in these constants to the TuningConstants Class: ",
            String.format(coefficients[1] + ", " + coefficients[0]));

        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }

        return data;
    }

    public String stringify(double[] array) {
        return String.format("%.3f", array[0]) + " " + String.format("%.3f", array[1]);
    }
}

// 0.00089x^2 + 0.07133x
//y = 0.00029x^2 + 0.10486x
// y = 0.00043x^2 + 0.09346x