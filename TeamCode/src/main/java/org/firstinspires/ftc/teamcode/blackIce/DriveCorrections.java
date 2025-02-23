package org.firstinspires.ftc.teamcode.blackIce;

import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;


public final class DriveCorrections {
    public static DriveCorrection adjustWithBrakingDistanceWithoutStop = () -> {
        double x = Target.xError - Odometry.xBrakingDistance;
        double y = Target.yError - Odometry.yBrakingDistance;

        // test without this because of early exit
        if (Math.abs(x) < 1) {
            x = Target.xError;
        }
        if (Math.abs(y) < 1) {
            y = Target.yError;
        }

        return fieldVectorToLocalWheelPowers(new double[]{x, y});
    };

    public static DriveCorrection stopAtTarget = () -> fieldVectorToLocalWheelPowers(
        new double[]{
            (Target.xError - Odometry.xBrakingDistance), // / ((double) 1 /4),  // try multiplying this whole thing
            (Target.yError - Odometry.yBrakingDistance) // / ((double) 1 /4),// / (1/4) inch error margin
        }
    );

    public static DriveCorrection stopAtTargetLateral = () -> {
        double[] robotVector = fieldVectorToRobotVector(new double[]{
            Target.xError, // - robot.odometry.xBrakingDistance) / ((double) 1 /4),  // try multiplying this whole thing
            Target.yError  // - robot.odometry.yBrakingDistance) / ((double) 1 /4),// / (1/4) inch error margin
        });
        return robotVectorToLocalWheelPowers(new double[] {
            robotVector[0] - Odometry.forwardBrakingDistance,
            robotVector[1] - Odometry.lateralBrakingDistance,
        });
    };

    public static DriveCorrection proportional = () -> fieldVectorToLocalWheelPowers(
        new double[]{
            Target.xError * 1.5,
            Target.yError * 1.5,
    });

    /**
     * Takes a field-relative vector and converts it into wheel powers
     * that would make the robot move in the direction of the field vector.
     * <p>
     * Does this by rotating the vector relative to the robot heading where it adds up the lateral
     * and forwards/backwards.
     */
    public static double[] fieldVectorToLocalWheelPowers(double[] vector) {
        return robotVectorToLocalWheelPowers(fieldVectorToRobotVector(vector));
    }

    public static double[] fieldVectorToRobotVector(double[] fieldVector) {
        // positive heading is counterclockwise
        double heading = Math.toRadians(Odometry.heading);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double localForwards = (fieldVector[0] * cos + fieldVector[1] * sin); // clockwise rotation
        double localSlide = (-fieldVector[0] * sin + fieldVector[1] * cos);
//        double localForwards = (x * cos + y * sin) / divisor - target.forwardBrakingDistance; // clockwise rotation
//        double localSlide = (-x * sin + y * cos) / divisor - target.lateralBrakingDistance;

        return new double[]{localForwards, localSlide};
    }

    public static double[] robotVectorToLocalWheelPowers(double[] robotVector) {
        double localForwards = robotVector[0];
        double localSlide = robotVector[1];

        return Util.normalize(new double[]
            {localForwards-localSlide, localForwards+localSlide,
             localForwards+localSlide, localForwards-localSlide}
        );
    }
}