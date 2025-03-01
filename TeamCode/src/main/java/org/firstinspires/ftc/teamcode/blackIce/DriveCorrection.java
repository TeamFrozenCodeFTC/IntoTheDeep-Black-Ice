package org.firstinspires.ftc.teamcode.blackIce;

import static org.firstinspires.ftc.teamcode.blackIce.Drive.fieldVectorToLocalWheelPowers;

import org.firstinspires.ftc.teamcode.blackIce.tuning.TuningConstants;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

public interface DriveCorrection {
    double[] calculateDrivePowers();

    DriveCorrection adjustWithBrakingDistanceWithoutStop = () -> {
        double x = Target.xError - Odometry.xBrakingDistance;
        double y = Target.yError - Odometry.yBrakingDistance;

        // test without this because of early exit
        if (Math.abs(x) < 1) {
            x = Target.xError; // * 3?
        }
        if (Math.abs(y) < 1) {
            y = Target.yError;
        }

        return fieldVectorToLocalWheelPowers(new double[]{x, y});
    };

    /**
     * Stop at the target using proportional control and predicting braking distance.
     * {@code (error * predictedBrakingDisplacement) * constant}
     */
    DriveCorrection stopAtTarget = () -> fieldVectorToLocalWheelPowers(
        new double[]{
            (Target.xError - Odometry.xBrakingDistance) * TuningConstants.HOLDING_PROPORTIONAL_CONSTANT,
            (Target.yError - Odometry.yBrakingDistance) * TuningConstants.HOLDING_PROPORTIONAL_CONSTANT
        }
    );

//    public static DriveCorrection stopAtTargetLateral = () -> {
//        double[] robotVector = fieldVectorToRobotVector(new double[]{
//            Target.xError, // - robot.odometry.xBrakingDistance) / ((double) 1 /4),  // try multiplying this whole thing
//            Target.yError  // - robot.odometry.yBrakingDistance) / ((double) 1 /4),// / (1/4) inch error margin
//        });
//        return robotVectorToLocalWheelPowers(new double[] {
//            robotVector[0] - Odometry.forwardBrakingDistance,
//            robotVector[1] - Odometry.lateralBrakingDistance,
//        });
//    };

    DriveCorrection proportional = () -> fieldVectorToLocalWheelPowers(
        new double[]{
            Target.xError * TuningConstants.PROPORTIONAL_CONSTANT,
            Target.yError * TuningConstants.PROPORTIONAL_CONSTANT,
        });
}
