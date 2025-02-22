package org.firstinspires.ftc.teamcode.blackIce.tuning;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;

public abstract class DistanceTuner extends Robot {
    public List<double[]> run(double heading, int points) {
        movement.target.setTarget(0, 0, 0);

        waitForStart();

        List<double[]> data = new ArrayList<>();

        for (int i = 0; i <= points; i++) {
            double percentageDone = (double) i / points;
            double power = Math.pow((1 - percentageDone), (double) 1/2);

            if (i % 2 == 0) {
                movement.buildMovement(heading, 48, 0)
                    .moveThrough()
                    .setMaxPower(power)
                    .run();
            }
            else {
                movement.buildMovement(heading, 0, 0)
                    .moveThrough()
                    .setMaxPower(power)
                    .run();
            }

            double startingX = odometry.x;
            double maxVelocity = Math.abs(odometry.xVelocity);

            drive.brakeFor(3);

            double newDistance = odometry.x;

            double brakingDistance = Math.abs(newDistance - startingX);

            data.add(new double[]{maxVelocity, brakingDistance});
            telemetry.addData(
                Integer.toString(i), stringify(new double[]{maxVelocity, brakingDistance}));
        }

        double[] coefficients = QuadraticRegression.quadraticFit(data);
        telemetry.addData(
            "Final Equation",
            String.format("y = %.5fx^2 + %.5fx + %.5f%n", coefficients[2], coefficients[1], coefficients[0]));

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