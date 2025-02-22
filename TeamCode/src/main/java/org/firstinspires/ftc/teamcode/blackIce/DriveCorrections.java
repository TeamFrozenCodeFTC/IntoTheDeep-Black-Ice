package org.firstinspires.ftc.teamcode.blackIce;


import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.tests.SquareRunTest;
import org.firstinspires.ftc.teamcode.util.Util;


public class DriveCorrections {
    Robot robot;

    public DriveCorrections(Robot robot) {
        this.robot = robot;
    }

    //private final Robot robot = Robot.instance;

    public double[] adjustWithBrakingDistanceWithoutStop() {
        double x = robot.movement.target.xError - robot.odometry.xBrakingDistance;
        double y = robot.movement.target.yError - robot.odometry.yBrakingDistance;

        // test without this because of early exit
        if (Math.abs(x) < 1) {
            x = robot.movement.target.xError;
        }
        if (Math.abs(y) < 1) {
            y = robot.movement.target.yError;
        }

        return new double[]{x, y};
    }
    public DriveCorrection adjustWithBrakingDistanceWithoutStop = this::adjustWithBrakingDistanceWithoutStop;


    public static double[] stopAtTarget() {
        return fieldVectorToLocalWheelPowers(new double[]{
            (robot.movement.target.xError - robot.odometry.xBrakingDistance), // / ((double) 1 /4),  // try multiplying this whole thing
            (robot.movement.target.yError - robot.odometry.yBrakingDistance) // / ((double) 1 /4),// / (1/4) inch error margin
        });
    }
    public DriveCorrection stopAtTarget = this::stopAtTarget;

    public static double[] stopAtTargetLateral() {
        double[] robotVector = fieldVectorToRobotVector(new double[]{
            robot.movement.target.xError, // - robot.odometry.xBrakingDistance) / ((double) 1 /4),  // try multiplying this whole thing
            robot.movement.target.yError  // - robot.odometry.yBrakingDistance) / ((double) 1 /4),// / (1/4) inch error margin
        });
        return robotVectorToLocalWheelPowers(new double[] {
            robotVector[0] - robot.odometry.forwardBrakingDistance,
            robotVector[1] - robot.odometry.lateralBrakingDistance,
        });
    }
    public DriveCorrection stopAtTargetLateral = this::stopAtTarget;

    public double[] proportional() {
        return fieldVectorToLocalWheelPowers(new double[]{
            robot.movement.target.xError,
            robot.movement.target.yError,
        });
    }
    public DriveCorrection proportional = this::proportional;
//
//    public double[] getWheelPowers(DriveCorrection driveCorrection) {
//        return fieldVectorToLocalWheelPowers(driveCorrection.calculateDriveVector());
//    }

    /**
     * Takes a field-relative vector and converts it into wheel powers
     * that would make the robot move in the direction of the field vector.
     * <p>
     * Does this by rotating the vector relative to the robot heading where it adds up the lateral
     * and forwards/backwards.
     */
    public double[] fieldVectorToLocalWheelPowers(double[] vector) {
        return robotVectorToLocalWheelPowers(fieldVectorToRobotVector(vector));
    }

    public double[] fieldVectorToRobotVector(double[] fieldVector) {
        // positive heading is counterclockwise
        double heading = Math.toRadians(robot.odometry.heading);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double localForwards = (fieldVector[0] * cos + fieldVector[1] * sin); // clockwise rotation
        double localSlide = (-fieldVector[0] * sin + fieldVector[1] * cos);
//        double localForwards = (x * cos + y * sin) / divisor - target.forwardBrakingDistance; // clockwise rotation
//        double localSlide = (-x * sin + y * cos) / divisor - target.lateralBrakingDistance;

        return new double[]{localForwards, localSlide};
    }

    public double[] robotVectorToLocalWheelPowers(double[] robotVector) {
        double localForwards = robotVector[0];
        double localSlide = robotVector[1];

        return Util.normalize(new double[]
            {localForwards-localSlide, localForwards+localSlide,
             localForwards+localSlide, localForwards-localSlide}
        );
    }
}