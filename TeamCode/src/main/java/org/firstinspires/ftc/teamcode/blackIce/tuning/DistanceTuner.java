package org.firstinspires.ftc.teamcode.blackIce.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackIce.Drive;
import org.firstinspires.ftc.teamcode.blackIce.Movement;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

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
     * */
    public static int POINTS = 10;

    public List<double[]> run(double heading) {
//        Odometry.setPosition(heading, 0, 0);

        waitForStart();

        Odometry.setPosition(heading, 0, 0);

        List<double[]> data = new ArrayList<>();

        for (int i = 0; i <= POINTS; i++) {
            double percentageDone = (double) i / POINTS;
            double power = Math.pow((1 - percentageDone), (double) 1/2);

            if (i % 2 == 0) {
                new Movement(48, 0, heading) // bug where previous heading doesn't work
                    .moveThrough()
                    .setMaxPower(power)
                    .waitForMovement();
            }
            else {
                new Movement(0, 0, heading)
                    .moveThrough()
                    .setMaxPower(power)
                    .waitForMovement();
            }

            Odometry.update();
            double startingX = Odometry.x;
            double maxVelocity = Math.abs(Odometry.xVelocity);

            Drive.brakeFor(3);

            Odometry.update();
            double newDistance = Odometry.x;

//            telemetry.addData("startingX", startingX);
//            telemetry.addData("maxVelocity", maxVelocity);
//            telemetry.addData("newDistanceX", newDistance);

            double brakingDistance = Math.abs(newDistance - startingX);

            data.add(new double[]{maxVelocity, brakingDistance});
            telemetry.addData(
                Integer.toString(i), stringify(new double[]{maxVelocity, brakingDistance}));

        }

        double[] coefficients = QuadraticRegression.quadraticFit(data);
        telemetry.addData(
            "Final Equation",
            String.format("y = %.5fx^2 + %.5fx + %.5f%n", coefficients[2], coefficients[1], coefficients[0]));
        telemetry.addData(
            "Plug in these constants to thee TuningConstants Class: ",
            String.format(coefficients[2] + ", ", coefficients[1] + ", " + coefficients[0]));

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